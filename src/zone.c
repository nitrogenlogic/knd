/*
 * zone.c - Zone checking and tracking.
 * Copyright (C)2011 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdlib.h>
#include <strings.h>

#include "knd.h"

const struct param_info param_ranges[] = {
	[ZONE_POP] = { .name = "pop", .min = 0, .max = FREENECT_FRAME_PIX, .def_rising = 160, .def_falling = 140 },
	[ZONE_SA] = { .name = "sa", .min = 0, .max = FREENECT_FRAME_PIX * 150, .def_rising = 3000, .def_falling = 1000 }, // mm^2
	[ZONE_BRIGHT] = { .name = "bright", .min = 0, .max = 1000, .def_rising = 350, .def_falling = 150 },
	[ZONE_XC] = { .name = "xc", .min = 0, .max = 1000, .def_rising = 600, .def_falling = 400 },
	[ZONE_YC] = { .name = "yc", .min = 0, .max = 1000, .def_rising = 600, .def_falling = 400 },
	[ZONE_ZC] = { .name = "zc", .min = 0, .max = 1000, .def_rising = 600, .def_falling = 400 },
};

/*
 * Converts x in pixels and z in world millimeters to x in world millimeters.
 */
static int xworld(int x, int zw)
{
	// tan 28 ~= .53171 (1089 ~= .53171 * 2048)
	// 0xcccd is the ~reciprocal of 10 (factor of W/2=320)
	// Add 2**34 (0x400000000) for rounding before shift
	// Shift right by 35:
	//   11 bits for tangent (* 2048 above)
	//   19 bits for reciprocal multiplication by 1/10 (factor of W/2=320)
	//   5 bits for division by 32 (other factor of W/2=320)
	//
	// In total, with an overly generous maximum range of +/-16384mm (15
	// bits), and an overhead of 35 bits due to arithmetic, 50 bits are
	// needed for the calculation.
	return (((int64_t)zw * (320 - x) * 1089 * 0xcccd) + 0x400000000) >> 35;
}

/*
 * Converts y in pixels and z in world millimeters to y in world millimeters.
 */
static int yworld(int y, int zw)
{
	return xworld(y + (FREENECT_FRAME_W - FREENECT_FRAME_H) / 2, zw);
}

/*
 * Converts x and z in world millimeters to x in pixels.
 */
static int xscreen(int xw, int zw)
{
	// Possible optimization: Get rid of division using reciprocal
	// multiplication.  This could be possible if depth_lut stores the
	// appropriate reciprocal and shift amount instead of/in addition to
	// the actual distance.  For the moment, xscreen is not called in any
	// performance-critical areas.
	return 320 - (((int64_t)xw << 35) / (1089 * 0xcccd * (int64_t)zw));
}

/*
 * Converts y and z in world millimeters to y in pixels.
 */
static int yscreen(int yw, int zw)
{
	return xscreen(yw, zw) - (FREENECT_FRAME_W - FREENECT_FRAME_H) / 2;
}

/*
 * Updates the given zone list's zone and depth maps.  Does not lock the zone
 * list.
 */
static void update_zone_map(struct zonelist *zones)
{
	int x, y, px;
	int i;

	// It would be faster to update only the sections of the map affected
	// by the zone that changed.

	for(y = 0; y < FREENECT_FRAME_H; y += zones->yskip) {
		px = y * FREENECT_FRAME_W;
		for(x = 0; x < FREENECT_FRAME_W; x += zones->xskip, px += zones->xskip) {
			zones->depth_map[px * 2] = UINT16_MAX;
			zones->depth_map[px * 2 + 1] = 0;

			for(i = 0; i < zones->count; i++) {
				struct zone *zone = zones->zones[i];
				if(zone->px_xmin <= x && zone->px_xmax >= x &&
						zone->px_ymin <= y && zone->px_ymax >= y) {
					// TODO: Use xworld/yworld to exclude zones from depth map,
					// since zones occupy fewer pixels at greater depths.
					if(zone->px_zmin < zones->depth_map[px * 2]) {
						zones->depth_map[px * 2] = zone->px_zmin;
					}
					if(zone->px_zmax > zones->depth_map[px * 2 + 1]) {
						zones->depth_map[px * 2 + 1] = zone->px_zmax;
					}
				}
			}
		}
	}

	zones->zone_map_dirty = 0;
}

/*
 * Updates the given zone list using the given depth image.
 */
void update_zonelist_depth(struct zonelist *zones, uint8_t *depthbuf)
{
	int x, y, z, px; // Screen-space x, y, depth, and pixel index
	int xw, yw, zw;
	int i, ret;
	int skip;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return;
	}

	if(zones->zone_map_dirty) {
		update_zone_map(zones);
	}

	zones->max_zone = -1;
	zones->occupied = 0;
	zones->oor_total = 0;
	skip = zones->xskip * zones->yskip;

	// Possible optimization: use a separate, contiguous array for all
	// dynamic zone data, so memset can be used to clear the entire list of
	// zones for every frame.
	for(i = 0; i < zones->count; i++) {
		zones->zones[i]->pop = 0;
		zones->zones[i]->xsum = 0;
		zones->zones[i]->ysum = 0;
		zones->zones[i]->zsum = 0;
	}

	for(y = 0; y < FREENECT_FRAME_H; y += zones->yskip) {
		px = y * FREENECT_FRAME_W;
		for(x = 0; x < FREENECT_FRAME_W; x += zones->xskip, px += zones->xskip) {
			z = pxval_11(depthbuf, px);
			if(z == 2047) {
				zones->oor_total += skip;
				continue;
			}

			// If the zones at this pixel are out of range for this pixel, continue.
			if(z < zones->depth_map[px * 2] || z > zones->depth_map[px * 2 + 1]) {
				continue;
			}

			zw = depth_lut[z];
			xw = xworld(x, zw);
			yw = yworld(y, zw);

			// TODO: Employ some of the optimizations mentioned in
			// knd.c to reduce the number of zone list iterations
			for(i = 0; i < zones->count; i++) {
				struct zone *zone = zones->zones[i];

				// TODO: Call custom shape function
				// TODO: Figure out if multiplication or bit
				// manipulation would be faster than the
				// conditional
				if(xw >= zone->xmin && xw <= zone->xmax &&
						yw >= zone->ymin && yw <= zone->ymax &&
						zw >= zone->zmin && zw <= zone->zmax) {
					zone->pop += skip;
					zone->xsum += skip * xw;
					zone->ysum += skip * yw;
					zone->zsum += skip * zw;
				}
			}
		}
	}

	int maxsa = 0;
	for(i = 0; i < zones->count; i++) {
		struct zone *zone = zones->zones[i];

		int sa = zone->pop > 0 ? (int)(zone->pop * surface_area((float)zone->zsum / zone->pop)) : 0;
		int threshold = zone->occupied ? zone->falling_threshold : zone->rising_threshold;
		int allow_occupied = zone->pop > 0;

		int param, occupied;

		switch(zone->occupied_param) {
			case ZONE_POP:
			default:
				param = zone->pop;
				break;

			case ZONE_SA:
				param = sa;
				break;

			case ZONE_BRIGHT:
				param = zone->bsum * 256 / zone->maxpop;
				allow_occupied = 1;
				break;

			case ZONE_XC:
				param = zone_xc(zone);
				break;

			case ZONE_YC:
				param = zone_yc(zone);
				break;

			case ZONE_ZC:
				param = zone_zc(zone);
				break;
		}

		occupied = allow_occupied && param >= threshold;

		if(zone->occupied != occupied) {
			zone->count++;
		} else {
			zone->count = 0;
		}

		if(!zone->occupied && zone->count > zone->rising_delay) {
			zone->occupied = 1;
			zone->count = 0;
		} else if(zone->occupied && zone->count > zone->falling_delay) {
			zone->occupied = 0;
			zone->count = 0;
		}

		zones->occupied += zone->occupied;

		if(sa > maxsa) {
			zones->max_zone = i;
			maxsa = sa;
		}
	}

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}
}

/*
 * Updates the given zone list using the given video image.
 */
void update_zonelist_video(struct zonelist *zones, uint8_t *videobuf)
{
	int x, y, px, b;
	int i, ret;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return;
	}

	if(zones->zone_map_dirty) {
		update_zone_map(zones);
	}

	// Possible optimization: use a separate, contiguous array for all
	// dynamic zone data, so memset can be used to clear the entire list of
	// zones for every frame.
	for(i = 0; i < zones->count; i++) {
		zones->zones[i]->bsum = 0;
	}

	// Only examine some of the green pixels from the Bayer image.
	for(y = 0; y < FREENECT_FRAME_H; y += 8) {
		px = y * FREENECT_FRAME_W;
		for(x = 1; x < FREENECT_FRAME_W; x += 8, px += 8) {
			b = videobuf[px];

			for(i = 0; i < zones->count; i++) {
				struct zone *zone = zones->zones[i];
				if(x >= zone->px_xmin && x <= zone->px_xmax &&
						y >= zone->px_ymin && y <= zone->px_ymax) {
					zone->bsum += b;
				}
			}
		}
	}

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}
}

/*
 * Creates an empty zone list.  One in every xskip columns and yskip rows will
 * be considered when the zone is updated.  Returns NULL on error.
 */
struct zonelist *create_zonelist(int xskip, int yskip)
{
	struct zonelist *zones;
	pthread_mutexattr_t mutex_attr;
	int ret;

	if((ret = pthread_mutexattr_init(&mutex_attr))) {
		ERROR_OUT("Error initializing mutex attributes: %s\n", strerror(ret));
		return NULL;
	}
	if((ret = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK))) {
		ERROR_OUT("Error setting mutex type to error checking: %s\n", strerror(ret));
		pthread_mutexattr_destroy(&mutex_attr);
		return NULL;
	}

	zones = calloc(1, sizeof(struct zonelist));
	if(zones == NULL) {
		ERRNO_OUT("Error allocating memory for zone list");
		pthread_mutexattr_destroy(&mutex_attr);
		return NULL;
	}

	if((ret = pthread_mutex_init(&zones->lock, &mutex_attr))) {
		ERROR_OUT("Error creating zone list mutex: %s\n", strerror(ret));
		free(zones);
		pthread_mutexattr_destroy(&mutex_attr);
		return NULL;
	}

	zones->xskip = xskip;
	zones->yskip = yskip;
	zones->max_zone = -1;

	pthread_mutexattr_destroy(&mutex_attr);
	return zones;
}

/*
 * Clears the given zone list without locking.
 */
static void clear_zonelist_nolock(struct zonelist *zones)
{
	int i;

	for(i = 0; i < zones->count; i++) {
		free(zones->zones[i]);
	}
	free(zones->zones);
	zones->zones = NULL;
	zones->count = 0;

	bump_zonelist_nolock(zones);
}

/*
 * Locks the given zone list and removes all of its zones.
 */
void clear_zonelist(struct zonelist *zones)
{
	int ret;

	if(CHECK_NULL(zones)) {
		return;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
	}

	clear_zonelist_nolock(zones);

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}
}

/*
 * Deallocates the given zone list, including all of its zones.
 */
void destroy_zonelist(struct zonelist *zones)
{
	int ret;

	if(CHECK_NULL(zones)) {
		return;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
	}

	clear_zonelist_nolock(zones);

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}

	if((ret = pthread_mutex_destroy(&zones->lock))) {
		ERROR_OUT("Error destroying zone list mutex: %s\n", strerror(ret));
	}

	free(zones);
}

/*
 * Locks the given zone list and calls cb for each zone in the list.  The zone
 * list should not be modified from within the callback.  The callback will not
 * be called if the list cannot be locked.
 */
void iterate_zonelist(struct zonelist *zones, zone_callback cb, void *cb_data)
{
	int i, ret;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return;
	}

	for(i = 0; i < zones->count; i++) {
		cb(cb_data, zones->zones[i]);
	}

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return;
	}
}

/*
 * Locks the given zone list, clears the new_zone flag, and updates lastpop and
 * lastoccupied for all zones.
 */
void touch_zonelist(struct zonelist *zones)
{
	int i, ret;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return;
	}

	for(i = 0; i < zones->count; i++) {
		zones->zones[i]->new_zone = 0;
		zones->zones[i]->lastpop = zones->zones[i]->pop;
		zones->zones[i]->lastoccupied = zones->zones[i]->occupied;
	}

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return;
	}
}

/*
 * Returns the number of zones in the given zone list in a thread-safe way.
 */
int zone_count(struct zonelist *zones)
{
	int count, ret;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
	}
	count = zones->count;
	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}

	return count;
}

/*
 * Returns the number of occupied zones in the given zone list.  Returns -1 on
 * error.
 */
int occupied_count(struct zonelist *zones)
{
	int occ;
	int ret;

	if(CHECK_NULL(zones)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	occ = zones->occupied;

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	return occ;
}

/*
 * Returns the name of the zone with the highest occupation.  The returned name
 * must be free()d.  If index, pop, and/or maxpop are not NULL, then the zone's
 * index, population, and screen area will be stored in *index, *pop, and
 * *maxpop.  Returns NULL and stores -1 if no zone is occupied.
 */
char *peak_zone(struct zonelist *zones, int *index, int *pop, int *maxpop)
{
	char *name = NULL;
	int idx = -1, p = -1, mp = -1;
	int ret;

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
	}
	if(zones->max_zone >= 0) {
		name = strdup(zones->zones[zones->max_zone]->name);
		idx = zones->max_zone;
		p = zones->zones[idx]->pop;
		mp = zones->zones[idx]->maxpop;
	}
	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
	}

	if(index) {
		*index = idx;
	}
	if(pop) {
		*pop = p;
	}
	if(maxpop) {
		*maxpop = mp;
	}

	return name;
}

/*
 * Adds a new rectangular zone to the given zone list.  Dimensions are in
 * world-space millimeters.  Returns a pointer to the new zone on success
 * (which may be passed to remove_zone()), NULL on error.
 */
struct zone *add_zone(struct zonelist *zones, char *name, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
	struct zone **tmp;
	struct zone *z;
	int i, ret;

	if(CHECK_NULL(zones) || CHECK_NULL(name)) {
		return NULL;
	}
	if(name[0] == 0) {
		ERROR_OUT("Name has zero length.\n");
		return NULL;
	}

	if(xmin >= xmax || ymin >= ymax || zmin >= zmax) {
		ERROR_OUT("Minimum must be < maximum.\n");
		return NULL;
	}

	if(zmin <= 0.0 || zmax <= 0.0) {
		ERROR_OUT("Z must be > 0.0.\n");
		return NULL;
	}

	if(strpbrk(name, "\r\n\t")) {
		ERROR_OUT("Name contains invalid characters.\n");
		return NULL;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return NULL;
	}

	for(i = 0; i < zones->count; i++) {
		if(!strcasecmp(name, zones->zones[i]->name)) {
			ERROR_OUT("Zone \"%s\" already exists.\n", name);
			pthread_mutex_unlock(&zones->lock);
			return NULL;
		}
	}

	z = calloc(1, sizeof(struct zone));
	if(z == NULL) {
		ERRNO_OUT("Error allocating memory for zone");
		pthread_mutex_unlock(&zones->lock);
		return NULL;
	}

	// Version is incremented by set_zone_nolock(), so don't increment in
	// this function.
	snprintf(z->name, sizeof(z->name), "%s", name);
	if(set_zone_nolock(zones, z, xmin, ymin, zmin, xmax, ymax, zmax)) {
		pthread_mutex_unlock(&zones->lock);
		free(z);
		return NULL;
	}

	z->occupied_param = ZONE_POP;
	z->rising_threshold = param_ranges[ZONE_POP].def_rising;
	z->falling_threshold = param_ranges[ZONE_POP].def_falling;
	z->rising_delay = 1;
	z->falling_delay = 1;

	tmp = realloc(zones->zones, sizeof(struct zone *) * (zones->count + 1));
	if(tmp == NULL) {
		ERRNO_OUT("Error growing zone list");
		pthread_mutex_unlock(&zones->lock);
		free(z);
		return NULL;
	}
	zones->zones = tmp;

	zones->zones[zones->count] = z;
	zones->count++;

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		free(z);
		return NULL;
	}

	return z;
}

/*
 * Recalculates the given zone's world coordinates ([xyz](min|max)) from its
 * screen coordinates (px_*).
 */
static void recalc_world_from_screen(struct zone *zone)
{
	// TODO: Try to make pixel-world-pixel conversions lossless
	// (right now there is single-"ulp" drift in some cases)
	zone->xmin = xworld(zone->px_xmax, (zone->px_xmax < FREENECT_FRAME_W / 2) ? zone->zmax : zone->zmin);
	zone->xmax = xworld(zone->px_xmin, (zone->px_xmin < FREENECT_FRAME_W / 2) ? zone->zmin : zone->zmax);
	zone->ymin = yworld(zone->px_ymax, (zone->px_ymax < FREENECT_FRAME_H / 2) ? zone->zmax : zone->zmin);
	zone->ymax = yworld(zone->px_ymin, (zone->px_ymin < FREENECT_FRAME_H / 2) ? zone->zmin : zone->zmax);
	zone->zmin = depth_lut[zone->px_zmin];
	zone->zmax = depth_lut[zone->px_zmax];
}

/*
 * Recalculates the given zone's screen coordinates (px_*) from its
 * world coordinates ([xyz](min|max)).
 */
static void recalc_screen_from_world(struct zone *zone)
{
	// TODO: Should these be clamped here, or at a higher level just before
	// display? See also TODO in set_zone_attr()
	zone->px_xmin = CLAMP(0, FREENECT_FRAME_W - 1, xscreen(zone->xmax, zone->xmax >= 0 ? zone->zmin : zone->zmax));
	zone->px_xmax = CLAMP(0, FREENECT_FRAME_W - 1, xscreen(zone->xmin, zone->xmin >= 0 ? zone->zmax : zone->zmin));
	zone->px_ymin = CLAMP(0, FREENECT_FRAME_H - 1, yscreen(zone->ymax, zone->ymax >= 0 ? zone->zmin : zone->zmax));
	zone->px_ymax = CLAMP(0, FREENECT_FRAME_H - 1, yscreen(zone->ymin, zone->ymin >= 0 ? zone->zmax : zone->zmin));
	zone->px_zmin = reverse_lut(zone->zmin);
	zone->px_zmax = reverse_lut(zone->zmax);
}

/*
 * Sets all base parameters on the given zone to the given values.  Does not
 * lock the zone list.  Only call this function if the zone list is already
 * locked.  Does increment the zone list version.  Returns 0 on success, -1 on
 * error.
 */
int set_zone_nolock(struct zonelist *zones, struct zone *zone, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
	if(CHECK_NULL(zones) || CHECK_NULL(zone)) {
		return -1;
	}

	if(xmin >= xmax || ymin >= ymax || zmin >= zmax) {
		ERROR_OUT("Minimum must be < maximum.\n");
		return -1;
	}

	if(zmin <= 0.0 || zmax <= 0.0) {
		ERROR_OUT("Z must be > 0.0.\n");
		return -1;
	}

	// Mark as a new zone so that new limits get sent to subscribers
	zone->new_zone = 1;

	zone->xmin = xmin;
	zone->xmax = xmax;
	zone->ymin = ymin;
	zone->ymax = ymax;
	zone->zmin = zmin;
	zone->zmax = zmax;

	recalc_screen_from_world(zone);

	// TODO: Treat zero-sized zones as an error?
	zone->maxpop = (zone->px_ymax - zone->px_ymin) * (zone->px_xmax - zone->px_xmin);
	if(zone->maxpop <= 0) {
		zone->maxpop = 1;
	}
	zone->lastpop = -1;
	zone->pop = 0;
	zone->occupied = 0;

	bump_zonelist_nolock(zones);

	return 0;
}

/*
 * Sets all base parameters on the given zone to the given values.  Locks the
 * given zone list.  Returns 0 on success, -1 on error.
 */
int set_zone(struct zonelist *zones, struct zone *zone, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
	int ret, result;

	if(CHECK_NULL(zones)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	result = set_zone_nolock(zones, zone, xmin, ymin, zmin, xmax, ymax, zmax);

	if(zones != NULL && (ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	return result;
}

/*
 * Sets the named attribute of the given zone to the given value.  A zone's
 * name, pop, maxpop, xc, yc, zc, sa, and occupied attributes may not be
 * changed.  Locks the zone list.  Returns 0 on success, -1 on error.
 */
int set_zone_attr(struct zonelist *zones, struct zone *zone, const char *attr, const char *value)
{
	enum { NONE, SCREEN, WORLD } recalc = NONE;
	int ival;
	int ret;

	if(CHECK_NULL(zone) || CHECK_NULL(attr) || CHECK_NULL(value)) {
		return -1;
	}

	if(!strcmp(value, "true")) {
		ival = 1;
	} else if(!strcmp(value, "false")) {
		ival = 0;
	} else {
		ival = atoi(value);
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	if(!strcmp(attr, "xmin")) {
		zone->xmin = ival;
		if(zone->xmax <= zone->xmin) {
			zone->xmax = zone->xmin + 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "xmax")) {
		zone->xmax = ival;
		if(zone->xmin >= zone->xmax) {
			zone->xmin = zone->xmax - 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "ymin")) {
		zone->ymin = ival;
		if(zone->ymax <= zone->ymin) {
			zone->ymax = zone->ymin + 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "ymax")) {
		zone->ymax = ival;
		if(zone->ymin >= zone->ymax) {
			zone->ymin = zone->ymax - 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "zmin")) {
		if(ival <= 0) {
			ERROR_OUT("Zmin must be > 0.0.\n");
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->zmin = ival;
		if(zone->zmax <= zone->zmin) {
			zone->zmax = zone->zmin + 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "zmax")) {
		if(ival <= 1) {
			ERROR_OUT("Zmax must be > 0.001.\n");
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->zmax = ival;
		if(zone->zmin >= zone->zmax) {
			zone->zmin = zone->zmax - 1;
		}
		recalc = SCREEN;
	} else if(!strcmp(attr, "px_xmin")) {
		if(ival < 0 || ival > FREENECT_FRAME_W - 2) {
			ERROR_OUT("px_xmin must be between 0 and %d\n", FREENECT_FRAME_W - 2);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->px_xmin = ival;
		if(zone->px_xmax <= zone->px_xmin) {
			zone->px_xmax = zone->px_xmin + 1;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "px_xmax")) {
		if(ival < 1 || ival > FREENECT_FRAME_W - 1) {
			ERROR_OUT("px_xmax must be between 1 and %d\n", FREENECT_FRAME_W - 1);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->px_xmax = ival;
		if(zone->px_xmin >= zone->px_xmax) {
			zone->px_xmin = zone->px_xmax - 1;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "px_ymin")) {
		if(ival < 0 || ival > FREENECT_FRAME_W - 2) {
			ERROR_OUT("px_ymin must be between 0 and %d\n", FREENECT_FRAME_W - 2);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->px_ymin = ival;
		if(zone->px_ymax <= zone->px_ymin) {
			zone->px_ymax = zone->px_ymin + 1;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "px_ymax")) {
		if(ival < 1 || ival > FREENECT_FRAME_W - 1) {
			ERROR_OUT("px_ymax must be between 1 and %d inclusive.\n", FREENECT_FRAME_W - 1);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->px_ymax = ival;
		if(zone->px_ymin >= zone->px_ymax) {
			zone->px_ymin = zone->px_ymax - 1;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "px_zmin")) {
		if(ival < 0 || ival > PXZMAX) {
			ERROR_OUT("px_zmin must be between 0 and %d inclusive.\n", PXZMAX);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}
		zone->px_zmin = ival;
		if(zone->px_zmax < zone->px_zmin) {
			zone->px_zmax = zone->px_zmin;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "px_zmax")) {
		if(ival < 0 || ival > PXZMAX) {
			ERROR_OUT("px_zmax must be between 0 and %d inclusive.\n", PXZMAX);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}
		zone->px_zmax = ival;
		if(zone->px_zmin > zone->px_zmax) {
			zone->px_zmin = zone->px_zmax;
		}
		recalc = WORLD;
	} else if(!strcmp(attr, "negate")) {
		if(ival != 0 && ival != 1) {
			ERROR_OUT("negate must be 0 or 1.\n");
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}
		zone->negate = ival;
		zone->occupied = zone->negate;
	} else if(!strcmp(attr, "param")) {
		int param;

		if(!strcmp(value, "pop")) {
			param = ZONE_POP;
		} else if(!strcmp(value, "sa")) {
			param = ZONE_SA;
		} else if(!strcmp(value, "bright")) {
			param = ZONE_BRIGHT;
		} else if(!strcmp(value, "xc")) {
			param = ZONE_XC;
		} else if(!strcmp(value, "yc")) {
			param = ZONE_YC;
		} else if(!strcmp(value, "zc")) {
			param = ZONE_ZC;
		} else {
			ERROR_OUT("Invalid zone control parameter: \"%s\"\n", value);
			pthread_mutex_unlock(&zones->lock);
			return -1;
		}

		zone->occupied_param = param;
		zone->occupied = 0;
		zone->count = 0;

		const struct param_info *range = &param_ranges[zone->occupied_param];
		zone->rising_threshold = range->def_rising;
		zone->falling_threshold = range->def_falling;
	} else if(!strcmp(attr, "on_level")) {
		const struct param_info *range = &param_ranges[zone->occupied_param];
		zone->rising_threshold = CLAMP(range->min, range->max, ival); // TODO: Decimal-to-int conversion for SA/XC/YC/ZC
		if(zone->falling_threshold > zone->rising_threshold) {
			zone->falling_threshold = zone->rising_threshold;
		}
	} else if(!strcmp(attr, "off_level")) {
		const struct param_info *range = &param_ranges[zone->occupied_param];
		zone->falling_threshold = CLAMP(range->min, range->max, ival); // TODO: Decimal-to-int for SA/XC/YC/ZC
		if(zone->rising_threshold < zone->falling_threshold) {
			zone->rising_threshold = zone->falling_threshold;
		}
	} else if(!strcmp(attr, "on_delay")) {
		zone->rising_delay = MAX_NUM(0, ival);
	} else if(!strcmp(attr, "off_delay")) {
		zone->falling_delay = MAX_NUM(0, ival);
	} else {
		ERROR_OUT("Unknown attribute: \"%s\"\n", attr);
		pthread_mutex_unlock(&zones->lock);
		return -1;
	}

	// TODO: Only calculate what's actually changed
	if(recalc == SCREEN) {
		recalc_screen_from_world(zone);
	} else if(recalc == WORLD) {
		// TODO: Deal with shrinking zones when adjusting px_* (maybe
		// don't change parameters for unchanged px_* at the edge of
		// the screen, and make sure frontend doesn't shrink zones by
		// one pixel when resizing)
		recalc_world_from_screen(zone);
	}

	zone->maxpop = (zone->px_ymax - zone->px_ymin) * (zone->px_xmax - zone->px_xmin);
	if(zone->maxpop <= 0) {
		zone->maxpop = 1;
	}
	zone->new_zone = 1;

	bump_zonelist_nolock(zones);

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	return 0;
}

/*
 * Frees the given zone.
 */
static void destroy_zone(struct zone *zone)
{
	// TODO: Handle shape information if/when added
	free(zone);
}

/*
 * Removes the given zone from the given zone list and frees its associated
 * resources.  Returns -1 if the zone was not found or zones is NULL, 0
 * otherwise.
 */
int remove_zone(struct zonelist *zones, struct zone *zone)
{
	int i, ret, origcount;
	struct zone **tmp;

	if(CHECK_NULL(zones) || CHECK_NULL(zone)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	origcount = zones->count;
	for(i = 0; i < zones->count; i++) {
		if(zones->zones[i] == zone) {
			destroy_zone(zones->zones[i]);

			if(i < zones->count - 1) {
				// TODO: Find out why first zone removal causes 2-byte invalid read in valgrind
				memmove(zones->zones + i, zones->zones + (i + 1), sizeof(struct zone *) * (zones->count - i));
			}

			tmp = realloc(zones->zones, sizeof(struct zone *) * (zones->count - 1));
			if(zones->count - 1 != 0 && tmp == NULL) {
				ERRNO_OUT("Error shrinking zone list");
			} else {
				zones->zones = tmp;
			}

			zones->count--;

			break;
		}
	}

	bump_zonelist_nolock(zones);

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	if(i == origcount) {
		ERROR_OUT("The given zone was not found in the given zone list.\n");
		return -1;
	}

	return 0;
}

/*
 * Finds the first zone with the given name.  Returns NULL if the zone wasn't
 * found or on error.
 */
struct zone *find_zone(struct zonelist *zones, const char *name)
{
	struct zone *z = NULL;
	int i, ret;

	if(CHECK_NULL(zones)) {
		return NULL;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return NULL;
	}

	for(i = 0; i < zones->count; i++) {
		if(!strcmp(zones->zones[i]->name, name)) {
			z = zones->zones[i];
			break;
		}
	}

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return NULL;
	}

	return z;
}

/*
 * Returns the version number of the given zone list.  The version number is
 * incremented every time a zone is added, removed, or modified.  Returns
 * (unsigned int)-1 on error.
 */
unsigned int get_zonelist_version(struct zonelist *zones)
{
	unsigned int version;
	int ret;

	if(CHECK_NULL(zones)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	version = zones->version;

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	return version;
}

/*
 * Increments the version number of the given zone list without locking or
 * error checking.  The version number is reset to zero if it reaches (unsigned
 * int)-1.
 */
unsigned int bump_zonelist_nolock(struct zonelist *zones)
{
	zones->zone_map_dirty = 1;

	zones->version++;
	if(zones->version == (unsigned int)-1) {
		zones->version = 0;
	}
	return zones->version;
}

/*
 * Locks and increments the version number of the given zone list.  The version
 * number will be reset to zero if it reaches (unsigned int)-1 (in other words,
 * the version can never be (unsigned int)-1).  Returns the new version number,
 * or (unsigned int)-1 on error.
 */
unsigned int bump_zonelist_version(struct zonelist *zones)
{
	unsigned int version;
	int ret;

	if(CHECK_NULL(zones)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&zones->lock))) {
		ERROR_OUT("Error locking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	version = bump_zonelist_nolock(zones);

	if((ret = pthread_mutex_unlock(&zones->lock))) {
		ERROR_OUT("Error unlocking zone list mutex: %s\n", strerror(ret));
		return -1;
	}

	return version;
}
