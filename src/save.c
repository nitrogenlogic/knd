/*
 * save.c - Zone saving
 * Copyright (C)2012 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <limits.h>

#include "knd.h"

// File format:
// file_version\n
// motor_tilt\n // Added in version 2
// zone_count\n
// name,xmin,ymin,zmin,xmax,ymax,zmax,param,on_level,off_level,on_delay,off_delay\n
// etc. (viewing angle changed in version 3, param..off_delay added in version 4, changed to integer mm in version 5)

#define ZONE_FORMAT 5 // File format version
#define ZONE_FILENAME "zones.knd"

struct save_info {
	struct knd_info *knd;

	struct nl_thread *thread;

	struct zonelist *zones;
	unsigned int last_version;

	char *savedir;

	struct timespec target;
	struct timespec interval;

	unsigned int stop:1;
};

// Returns 1 if file/dir exists, 0 if it does not, -errno on error.
static int exists(const char *filename)
{
	struct stat statbuf;

	if(stat(filename, &statbuf)) {
		if(errno == ENOENT) {
			return 0;
		}
		return -errno;
	}

	return 1;
}

// Returns 1 if path is a directory, 0 otherwise, -errno on error.
static int isdir(const char *pathname)
{
	struct stat statbuf;

	if(stat(pathname, &statbuf)) {
		if(errno == ENOENT) {
			return 0;
		}
		return -errno;
	}

	return !!S_ISDIR(statbuf.st_mode);
}

// Returns 1 if the given gid_t value is in the given array, 0 otherwise.
static int isin(gid_t value, gid_t *array, int size)
{
	int i;

	for(i = 0; i < size; i++) {
		if(array[i] == value) {
			return 1;
		}
	}

	return 0;
}

// Returns 1 if the current user has all the given privileges (use S_I?USR), 0
// otherwise, -errno on error.
static int check_statbit(struct stat *stbuf, mode_t bits)
{
	gid_t groups[NGROUPS_MAX]; // 256KB of stack eaten.  Tasty.
	int groupcount;

	if((groupcount = getgroups(ARRAY_SIZE(groups), groups)) < 0) {
		ERRNO_OUT("Error getting current user's group membership");
		return -errno;
	}

	bits &= S_IRWXU;

	if(stbuf->st_uid == geteuid()) {
		// World or group permissions are useless if uid matches
		return (stbuf->st_mode & bits) == bits;
	}

	// Assuming S_I?GRP is 3 bits right of S_I?USR
	bits >>= 3;
	if(stbuf->st_gid == getegid() || isin(stbuf->st_gid, groups, groupcount)) {
		// World permissions are useless if gid matches
		return (stbuf->st_mode & bits) == bits;
	}

	bits >>= 3;
	return (stbuf->st_mode & bits) == bits;
}

// Returns 1 if path is writable (and executable if a directory), 0 otherwise,
// -errno on error.
static int canwrite(const char *pathname)
{
	struct stat statbuf;
	int wr;

	if(stat(pathname, &statbuf)) {
		if(errno == ENOENT) {
			return 0;
		}
		return -errno;
	}

	wr = check_statbit(&statbuf, S_IWUSR);

	if(wr && S_ISDIR(statbuf.st_mode)) {
		return check_statbit(&statbuf, S_IXUSR);
	}

	return wr;
}

static void *save_thread(void *data)
{
	struct save_info *info = (struct save_info *)data;

	while(!info->stop) {
		usleep(500000 + rand() % 100000);
		check_save(info);
	}

	return NULL;
}

/*
 * Initializes zone saving (makes sure the directory exists and is writable)
 * and starts a periodic saving background thread.  This should not be called
 * while the given zone list is being accessed by another thread.  Returns a
 * pointer to a save_info struct on success, NULL on error.
 */
struct save_info *init_save(struct knd_info *knd, struct zonelist *zones, const char *savedir, struct timespec *interval)
{
	struct save_info *info;
	int ex, dir;
	int ret;

	if(CHECK_NULL(knd) || CHECK_NULL(zones) || CHECK_NULL(savedir)) {
		return NULL;
	}

	// savedir length + filename length + directory separator
	if(strlen(savedir) + strlen(ZONE_FILENAME) + 1 >= PATH_MAX) {
		ERROR_OUT("Save location '%s' is too long.\n", savedir);
		return NULL;
	}

	ex = exists(savedir);
	dir = isdir(savedir);

	if(ex < 0) {
		ERROR_OUT("Error checking the existence of save location '%s': %s\n",
				savedir, strerror(-ex));
		return NULL;
	}
	if(dir < 0) {
		ERROR_OUT("Error checking whether save location '%s' is a directory: %s\n",
				savedir, strerror(-dir));
		return NULL;
	}

	if(!ex) {
		ERROR_OUT("Save location '%s' does not exist.\n", savedir);
		return NULL;
	}
	if(ex && !dir) {
		ERROR_OUT("Save location '%s' is not a directory.\n", savedir);
		return NULL;
	}

	if(!canwrite(savedir)) {
		ERROR_OUT("Save location '%s' is not writable.\n", savedir);
		return NULL;
	}

	info = calloc(1, sizeof(struct save_info));
	if(info == NULL) {
		ERRNO_OUT("Error allocating memory for zone saving information");
		return NULL;
	}

	info->savedir = strdup(savedir);
	if(info->savedir == NULL) {
		ERRNO_OUT("Error duplicating save location string");
		free(info);
		return NULL;
	}
	info->knd = knd;
	info->zones = zones;
	info->last_version = get_zonelist_version(zones);
	info->interval = *interval;
	clock_gettime(CLOCK_MONOTONIC, &info->target);

	ret = nl_create_thread(knd->thread_ctx, NULL, save_thread, info, "save_thread", &info->thread);
	if(ret) {
		ERROR_OUT("Error starting zone saving thread: %d (%s)\n", ret, strerror(ret));
		cleanup_save(info);
		return NULL;
	}

	return info;
}

/*
 * Frees the given save information and stops the associated thread.  Call this
 * *before* the associated zone list is freed.
 */
void cleanup_save(struct save_info *info)
{
	int ret;

	if(CHECK_NULL(info)) {
		return;
	}

	if(info->thread != NULL) {
		info->stop = 1;
		ret = nl_join_thread(info->thread, NULL);
		if(ret) {
			ERROR_OUT("Error joining zone saving thread: %d (%s)\n", ret, strerror(ret));
		}
	}
	if(info->savedir != NULL) {
		free(info->savedir);
	}
	free(info);
}

/*
 * Writes a single zone's info to the given file.
 */
static void save_zone_callback(void *data, struct zone *zone)
{
	FILE *output = data;

	// TODO: Escape names if commas are ever permitted or format changes
	fprintf(output, "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			zone->name,
			zone->xmin, zone->ymin, zone->zmin,
			zone->xmax, zone->ymax, zone->zmax,
			zone->occupied_param, zone->rising_threshold, zone->falling_threshold,
			zone->rising_delay, zone->falling_delay);
}

/*
 * Unconditionally saves the zone list associated with the given save info.
 * First saves into a temporary file in the same directory, then uses rename()
 * to overwrite the original file.
 * Returns 0 on success, -1 on error.
 */
int save_zones(struct save_info *info)
{
	FILE *output;
	char tmppath[PATH_MAX + 1];
	char path[PATH_MAX + 1];
	size_t len;
	int ret = 0;

	len = snprintf(tmppath, ARRAY_SIZE(tmppath), "%s/%s.tmp", info->savedir, ZONE_FILENAME);
	if(len >= ARRAY_SIZE(tmppath)) {
		ERROR_OUT("Save filename and path is too long.");
		return -1;
	}
	snprintf(path, ARRAY_SIZE(path), "%s/%s", info->savedir, ZONE_FILENAME);

	// TODO: Make sure tmppath isn't a hardlink to path (indicating an
	// interrupted rename())?

	output = fopen(tmppath, "wt");
	if(output == NULL) {
		ERRNO_OUT("Error opening zone save file '%s' for writing", tmppath);
		return -1;
	}

	// Here's where exceptions would be nice for handling a full
	// filesystem, bad sector, or other error
	if(fprintf(output, "%d\n", ZONE_FORMAT) < 0 ||
			fprintf(output, "%d\n", get_tilt(info->knd->vid)) < 0 ||
			fprintf(output, "%d\n", zone_count(info->zones)) < 0) {
		ERRNO_OUT("Error writing zone save file header to '%s'", tmppath);
		fclose(output);
		return -1;
	}

	// FIXME: Zone list may have been modified between getting the count
	// above and this code running.  Maybe a flag could be used to prevent
	// accidental concurrent modification, where a function that needs
	// atomic access calls lock_zonelist(), which sets the flag within the
	// standard zone lock.
	iterate_zonelist(info->zones, save_zone_callback, output);

	if(fflush(output) == EOF) {
		ERRNO_OUT("Error flushing zone save file '%s'", tmppath);
		ret = -1;
	}
	if(ferror(output)) {
		ERROR_OUT("An I/O error occurred while saving zones to '%s'.\n", tmppath);
		ret = -1;
	}
	if(fsync(fileno(output))) {
		ERRNO_OUT("Error syncing zone save file '%s' to disk", path);
		ret = -1;
	}
	if(fclose(output) == EOF) {
		ERRNO_OUT("Error closing zone save file '%s'", path);
		ret = -1;
	}
	if(rename(tmppath, path)) {
		ERRNO_OUT("Error renaming zone save file '%s' to '%s'", tmppath, path);
		ret = -1;
	}

	if(ret == 0) {
		info->last_version = get_zonelist_version(info->zones);
	}

	return ret;
}

/*
 * Loads zone information, if it exists, from the directory pointed to by info.
 * Does not remove any existing zones from the zone list.  Returns number of
 * zones read on success, -1 on error.
 */
int load_zones(struct save_info *info)
{
	FILE *input;
	char path[PATH_MAX];
	char name[ZONE_NAME_LENGTH];
	struct zone *z;
	float fl_xmin, fl_ymin, fl_zmin, fl_xmax, fl_ymax, fl_zmax;
	int xmin, ymin, zmin, xmax, ymax, zmax;
	int param, rising_threshold, falling_threshold, rising_delay, falling_delay;
	unsigned int version;
	int filever, count;
	int tilt;
	int i;

	snprintf(path, ARRAY_SIZE(path), "%s/%s", info->savedir, ZONE_FILENAME);

	input = fopen(path, "rt");
	if(input == NULL) {
		ERRNO_OUT("Error opening zone save file '%s' for reading", path);
		return -1;
	}

	if(fscanf(input, "%d\n", &filever) != 1) {
		ERRNO_OUT("Error reading zone file version from '%s'", path);
		fclose(input);
		return -1;
	}

	if(filever < 1 || filever > ZONE_FORMAT) {
		ERROR_OUT("Zone file version %d is unsupported (only versions 1-%d are supported).\n",
				filever, ZONE_FORMAT);
		fclose(input);
		return -1;
	}

	if(filever >= 2) {
		if(fscanf(input, "%d\n", &tilt) != 1) {
			ERRNO_OUT("Error reading motor tilt from '%s'", path);
		} else {
			set_tilt(info->knd->vid, tilt);
		}
	}

	if(filever < 3) {
		nl_ptmf("Converting zones to new viewing angle.\n");
	}

	if(fscanf(input, "%d\n", &count) != 1) {
		ERRNO_OUT("Error reading zone count from '%s'", path);
		fclose(input);
		return -1;
	}

	for(i = 0; !feof(input); i++) {
		name[0] = 0;
		if(filever < 4) {
			if(fscanf(input, "%127[^,],%f,%f,%f,%f,%f,%f\n", name,
						&fl_xmin, &fl_ymin, &fl_zmin, &fl_xmax, &fl_ymax, &fl_zmax) != 7) {
				ERROR_OUT("Error reading zone %d ('%s') from '%s': invalid zone format.\n",
						i + 1, name, path);
				continue;
			}

			if(filever < 3) {
				// Scale to new viewing angle (* tan(28)/tan(35))
				const float scale = .759359765;
				xmin *= scale;
				xmax *= scale;
				ymin *= scale;
				ymax *= scale;
			}
		} else if(filever < 5) {
			if(fscanf(input, "%127[^,],%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n", name,
						&fl_xmin, &fl_ymin, &fl_zmin, &fl_xmax, &fl_ymax, &fl_zmax,
						&param, &rising_threshold, &falling_threshold,
						&rising_delay, &falling_delay) != 12) {
				ERROR_OUT("Error reading zone %d ('%s') from '%s': invalid zone format.\n",
						i + 1, name, path);
				continue;
			}
		} else {
			if(fscanf(input, "%127[^,],%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", name,
						&xmin, &ymin, &zmin, &xmax, &ymax, &zmax,
						&param, &rising_threshold, &falling_threshold,
						&rising_delay, &falling_delay) != 12) {
				ERROR_OUT("Error reading zone %d ('%s') from '%s': invalid zone format.\n",
						i + 1, name, path);
				continue;
			}
		}

		// Convert float values to integers
		if(filever < 5) {
			xmin = (int)(fl_xmin * 1000);
			ymin = (int)(fl_ymin * 1000);
			zmin = (int)(fl_zmin * 1000);
			xmax = (int)(fl_xmax * 1000);
			ymax = (int)(fl_ymax * 1000);
			zmax = (int)(fl_zmax * 1000);
		}

		if(xmin == xmax) {
			xmax = xmin + 100;
		}
		if(ymin == ymax) {
			ymax = ymin + 100;
		}
		if(zmin == zmax) {
			zmax = zmin + 100;
		}

		z = add_zone(info->knd->zones, name, xmin, ymin, zmin, xmax, ymax, zmax);
		if(z == NULL) {
			ERROR_OUT("Error adding zone %d ('%s') from '%s' to the zone list.\n",
					i + 1, name, path);
			continue;
		}

		if(filever >= 4) {
			z->occupied_param = param;
			z->rising_threshold = rising_threshold;
			z->falling_threshold = falling_threshold;
			z->rising_delay = rising_delay;
			z->falling_delay = falling_delay;
		}
	}
	if(i != count) {
		ERROR_OUT("Zone count mismatch in '%s': read %d zones, expected %d.\n",
				path, i, count);
		count = i;
	}

	if(ferror(input)) {
		ERROR_OUT("An I/O error occurred while loading zones from '%s'.\n", path);
		fclose(input);
		return -1;
	}

	version = get_zonelist_version(info->zones);
	if(version == (unsigned int)-1) {
		ERROR_OUT("Error getting zone list version.\n");
	} else {
		info->last_version = version;
	}

	fclose(input);
	return count;
}

/*
 * Saves the zone list associated with this save info if its save interval has
 * elapsed since the last save and the zone list's version has changed.  On the
 * off chance that approximately (2^32)-1 zone changes occur between calls to
 * this function, the zones will not be saved.  This function should not be
 * called while the zone list is locked.  Returns 1 if the zones were not
 * saved, 0 on successful save, -1 on error.
 */
int check_save(struct save_info *info)
{
	struct timespec now;
	unsigned int version;

	clock_gettime(CLOCK_MONOTONIC, &now);

	if(NL_TIMESPEC_GTE(info->target, now)) {
		return 1;
	}

	version = get_zonelist_version(info->zones);
	if(version == (unsigned int)-1) {
		ERROR_OUT("Error getting zone list version for saving zones.\n");
		return -1;
	}

	if(version == info->last_version) {
		return 1;
	}

	info->target = nl_add_timespec(now, info->interval);

	nl_ptmf("Saving zones.\n");
	return save_zones(info);
}
