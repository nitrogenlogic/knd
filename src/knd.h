/*
 * knd.h - Depth camera daemon program-wide definitions.
 * Copyright (C)2012 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#ifndef _KND_H_
#define _KND_H_

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>
#include <pthread.h>

#include <libfreenect.h>

#include <nlutils/nlutils.h>

#ifndef FREENECT_FRAME_W // TODO: Use dynamic resolution
# define FREENECT_FRAME_W 640
# define FREENECT_FRAME_H 480
# define FREENECT_FRAME_PIX 307200
#endif /* FREENECT_FRAME_W */

#ifndef FREENECT_DEPTH_11BIT_PACKED_SIZE
# define FREENECT_DEPTH_11BIT_PACKED_SIZE 422400
#endif /* FREENECT_DEPTH_11BIT_PACKED_SIZE */

#ifndef KND_INLINE
# define KND_INLINE inline
#endif /* KND_INLINE */

#ifndef KND_PORT
# define KND_PORT 14308
#endif /* KND_PORT */

#define ZONE_NAME_LENGTH 128

#define PXZMAX 1092

#define KND_DEPTH_SIZE FREENECT_DEPTH_11BIT_PACKED_SIZE

// TODO: Allow runtime image format selection, or create a getir command?
#define KND_VIDEO_FORMAT FREENECT_VIDEO_BAYER // FREENECT_VIDEO_IR_8BIT
#define KND_VIDEO_SIZE (640*480) // FREENECT_VIDEO_IR_8BIT_SIZE


struct knd_watchdog;
struct vidproc_info;
struct zone;
struct zonelist;
struct knd_server;
struct knd_client;
struct save_info;

/*
 * Server/program state.
 */
struct knd_info {
	struct nl_thread_ctx *thread_ctx;

	struct knd_watchdog *wd;
	struct vidproc_info *vid;
	struct knd_server *srv;

	// Framerate tracking (TODO: locking to make drd/helgrind happy)
	int frames;
	int fps;
	struct timespec last_time;
	struct timespec next_time;

	struct zonelist *zones; // Global zone list
	struct save_info *save; // Zone-saving info for the global zone list

	volatile unsigned int stop:1;	  // Set to 1 to stop main loop
	volatile unsigned int crashing:1; // Set to 1 if a crash handler has been called
};

/*
 * Zone definition.
 */
struct zone {
	char name[ZONE_NAME_LENGTH];
	unsigned int new_zone:1; // 1 if not yet sent by subscriptions

	// Bounding box (dimensions in world-space millimeters)
	int xmin, xmax;
	int ymin, ymax;
	int zmin, zmax;

	// On-screen bounding box (in pixels and nonlinear depth units)
	int px_xmin, px_xmax;
	int px_ymin, px_ymax;
	int px_zmin, px_zmax;

	// Zone population
	int maxpop;
	int lastpop;
	int pop;
	// For center of gravity (overflow shouldn't be possible with
	// 640x480x1000x10 possible contributing units)
	int xsum;
	int ysum;
	unsigned int zsum;

	unsigned int occupied:1;
	unsigned int lastoccupied:1;

	// Custom zone logic
	unsigned int negate:1; // Reverse occupied flag if true
	enum {
		ZONE_POP,
		ZONE_SA,
		ZONE_BRIGHT,
		ZONE_XC,
		ZONE_YC,
		ZONE_ZC,
	} occupied_param;
	int rising_threshold;
	int falling_threshold;
	int rising_delay; // Require rising_delay/falling_delay continuous frames
	int falling_delay;
	int count; // Number of frames seen so far while waiting for *_delay

	// For brightness from video
	int bsum;

	// For future implementation of custom shapes
	void *shape_data;
	int (*contains)(int xw, int yw, int zw); // world-space mm
	int (*may_contain)(int x, int y, int z); // pixels
};

/*
 * List of zones.
 */
struct zonelist {
	pthread_mutex_t lock;

	// Even indices: minimum depth of any zone at that pixel, odd: max.
	uint16_t depth_map[640*480*2];

	// Set to 1 when the zone and depth maps need to be updated.
	unsigned int zone_map_dirty:1;

	struct zone **zones;
	int count;
	unsigned int version; // Overflow is okay if versions are assumed to be unordered

	int xskip;
	int yskip;

	int max_zone; // Zone with highest population (-1 if no zones)
	int occupied; // Number of occupied zones
	int oor_total;
};

/*
 * Stores acceptable range, parameter name, and default values for
 * rising/falling thresholds based on parameter.
 */
struct param_info {
	char name[16];
	int min;
	int max;
	int def_rising;
	int def_falling;
};


/***** knd.c *****/


/***** watchdog.c *****/

/*
 * Callback called on watchdog timeout.  The data given to create_watchdog() is
 * passed in data.  The time since the last watchdog kick is in interval.
 */
typedef void (*watchdog_func)(void *data, struct timespec *interval);

/*
 * Creates a new watchdog thread.  kick_watchdog() must be called every
 * timeout nsecs or faster after this function returns.  The watchdog will be
 * checked for a timeout every interval nsecs.  Returns the newly-created
 * watchdog on success, NULL on error.  Note that C99 compound literals may be
 * used for the interval and timeout parameters.
 */
struct knd_watchdog *create_watchdog(struct knd_info *knd, struct timespec *interval, struct timespec *timeout, void *data, watchdog_func callback);

/*
 * Stops a running watchdog (waiting for it to exit) and frees its associated
 * resources.  Takes no action if wd is NULL.
 */
void destroy_watchdog(struct knd_watchdog *wd);

/*
 * Resets the given watchdog's timeout countdown.  Doesn't check for a NULL
 * watchdog.
 */
void kick_watchdog(struct knd_watchdog *wd);

/*
 * Sets the given watchdog's timeout in a thread-safe way.  Does not kick the
 * watchdog.
 */
void set_watchdog_timeout(struct knd_watchdog *wd, struct timespec *timeout);


/***** vidproc.c *****/

/*
 * Callbacks for video processing functions, typically called from a separate
 * video processing thread.
 */
typedef void (*vidproc_func)(uint8_t *buffer, void *data);

/*
 * Depth look-up table (translates depth sample into world-space millimeters).
 */
extern int depth_lut[2048];

/*
 * Surface area look-up table (translates depth sample into world-space surface
 * area of a pixel at that distance).
 */
float surface_lut[2048];

/*
 * Returns the surface area of a single pixel at the given distance.  Works for
 * any unit (mm->mm^2, m->m^2, etc.).  Does not use the surface area look-up
 * table.
 */
float surface_area(float z);

/*
 * Initializes the depth look-up table.
 */
void init_lut();

/*
 * Initializes libfreenect and opens the devindex-th camera.  If depth_cb
 * and/or video_cb are not NULL, then they will be called for every frame
 * received and processed.  Callbacks are called from the vidproc thread after
 * vidproc's own processing is complete, so use appropriate locking and avoid
 * spending too much time in callbacks.
 */
struct vidproc_info *init_vidproc(struct knd_info *knd, int devindex, vidproc_func depth_cb, void *depth_cb_data, vidproc_func video_cb, void *video_cb_data);

/*
 * Stops video processing associated with the given info structure and frees
 * associated resources.  Should only be called after the libfreenect event
 * loop has exited. Ignores a null parameter.
 */
void cleanup_vidproc(struct vidproc_info *info);

/*
 * Runs one iteration of libfreenect event processing.  This function blocks
 * until there are events to process, so it should be called through its own
 * event loop.
 */
int vidproc_doevents(struct vidproc_info *info);

/*
 * Locks the depth buffer and calls the given callback once with a pointer to
 * the depth buffer.  Also schedules an update of the camera's LED to indicate
 * video recording.  Returns 0 on success, -1 on error.
 */
int get_depth(struct vidproc_info *info, vidproc_func cb, void *cb_data);

/*
 * Starts video processing to grab a single frame of video (the video callback
 * will stop video processing after receiving a single frame).  Sets the video
 * requested flag, so that multiple calls to request_video() without calling
 * get_video() will start video processing only once.  Returns 0 on success, -1
 * on error.
 */
int request_video(struct vidproc_info *info);

/*
 * Locks the video buffer and calls the given callback once with a pointer to
 * the video buffer.  Also schedules an update of the camera's LED to indicate
 * video recording.  Returns 0 on success, -1 on error.
 */
int get_video(struct vidproc_info *info, vidproc_func cb, void *cb_data);

/*
 * Finds the closest entry in the depth look-up table to the given world-space
 * depth value in millimeters without going over.  Uses a binary search.
 */
int reverse_lut(int zw);

/*
 * Returns the currently-requested motor tilt in degrees from horizontal.  The
 * motor's actual current position may be different.
 */
int get_tilt(struct vidproc_info *info);

/*
 * Requests that the motor tilt the camera to the specified number of degrees
 * from horizontal.  Takes no action if the motor device was not opened.
 */
void set_tilt(struct vidproc_info *info, int tilt);

/*
 * Unpacks the pixel-th 11-bit pixel from the given packed buffer.
 */
KND_INLINE int pxval_11(const uint8_t *buf, int pixel)
{
	uint32_t byteindex = (pixel * 11) >> 3;
	uint32_t shiftbits = ((7 + pixel * 5) & 0x7) + 14;
	uint32_t base;

	buf = buf + byteindex;
	base = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

	return (base >> shiftbits) & 0x7ff;
}


/***** zone.c *****/

/*
 * Callback function used by iterate_zonelist().
 */
typedef void (*zone_callback)(void *data, struct zone *zone);

/*
 * Updates the given zone list using the given depth image.
 */
void update_zonelist_depth(struct zonelist *zones, uint8_t *depthbuf);

/*
 * Updates the given zone list using the given video image.
 */
void update_zonelist_video(struct zonelist *zones, uint8_t *videobuf);

/*
 * Creates an empty zone list.  One in every xskip columns and yskip rows will
 * be considered when the zone is updated.  Returns NULL on error.
 */
struct zonelist *create_zonelist(int xskip, int yskip);

/*
 * Locks the given zone list and removes all of its zones.
 */
void clear_zonelist(struct zonelist *zones);

/*
 * Deallocates the given zone list, including all of its zones.
 */
void destroy_zonelist(struct zonelist *zones);

/*
 * Locks the given zone list and calls cb for each zone in the list.  The zone
 * list should not be modified from within the callback.  The callback will not
 * be called if the list cannot be locked.
 */
void iterate_zonelist(struct zonelist *zones, zone_callback cb, void *cb_data);

/*
 * Locks the given zone list, clears the new_zone flag, and updates lastpop and
 * lastoccupied for all zones.
 */
void touch_zonelist(struct zonelist *zones);

/*
 * Returns the number of zones in the given zone list in a thread-safe way.
 */
int zone_count(struct zonelist *zones);

/*
 * Returns the number of occupied zones in the given zone list.  Returns -1 on
 * error.
 */
int occupied_count(struct zonelist *zones);

/*
 * Returns the name of the zone with the highest occupation.  The returned name
 * must be free()d.  If index, pop, and/or maxpop are not NULL, then the zone's
 * index, population, and screen area will be stored in *index, *pop, and
 * *maxpop.  Returns NULL and stores -1 if no zone is occupied.
 */
char *peak_zone(struct zonelist *zones, int *index, int *pop, int *maxpop);

/*
 * Adds a new rectangular zone to the given zone list.  Dimensions are in
 * world-space millimeters.  Returns a pointer to the new zone on success
 * (which may be passed to remove_zone()), NULL on error.
 */
struct zone *add_zone(struct zonelist *zones, char *name, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);

/*
 * Sets all base parameters on the given zone to the given values.  Does not
 * lock the zone list.  Only call this function if the zone list is already
 * locked.  Does increment the zone list version.  Returns 0 on success, -1 on
 * error.
 */
int set_zone_nolock(struct zonelist *zones, struct zone *zone, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);

/*
 * Sets all base parameters on the given zone to the given values.  Locks the
 * given zone list.  Returns 0 on success, -1 on error.
 */
int set_zone(struct zonelist *zones, struct zone *zone, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);

/*
 * Sets the named attribute of the given zone to the given value.  A zone's
 * name, pop, maxpop, xc, yc, zc, sa, and occupied attributes may not be
 * changed.  Locks the zone list.  Returns 0 on success, -1 on error.
 */
int set_zone_attr(struct zonelist *zones, struct zone *zone, const char *attr, const char *value);

/*
 * Removes the given zone from the given zone list and frees its associated
 * resources.  Returns -1 if the zone was not found or zones is NULL, 0
 * otherwise.
 */
int remove_zone(struct zonelist *zones, struct zone *zone);

/*
 * Finds the first zone with the given name.  Returns NULL if the zone wasn't
 * found or on error.
 */
struct zone *find_zone(struct zonelist *zones, const char *name);

/*
 * Returns the version number of the given zone list.  The version number is
 * incremented every time a zone is added, removed, or modified.  Returns
 * (unsigned int)-1 on error.
 */
unsigned int get_zonelist_version(struct zonelist *zones);

/*
 * Increments the version number of the given zone list without locking or
 * error checking.  The version number is reset to zero if it reaches (unsigned
 * int)-1.
 */
unsigned int bump_zonelist_nolock(struct zonelist *zones);

/*
 * Locks and increments the version number of the given zone list.  The version
 * number will be reset to zero if it reaches (unsigned int)-1 (in other words,
 * the version can never be (unsigned int)-1).  Returns the new version number,
 * or (unsigned int)-1 on error.
 */
unsigned int bump_zonelist_version(struct zonelist *zones);

/*
 * Information about parameters available for a zone's occupation detection
 * parameter.
 */
extern const struct param_info param_ranges[];

/*
 * Calculates the proportional X-axis center of gravity for the given zone.
 * The return value will range from 0 to 1000 if the zone's population is
 * greater than 0.  Returns -1 otherwise.
 */
KND_INLINE int zone_xc(struct zone *zone)
{
	// Rearranging the operations and using 64-bit integers could give
	// 3 multiplies and 1 divide instead of 1 multiply and 2 divides
	return zone->pop > 0 ?
		(int)((zone->xsum / zone->pop - zone->xmin) * 1000 / (zone->xmax - zone->xmin)) :
		-1;
}

/*
 * Calculates the proportional Y-axis center of gravity for the given zone.
 * The return value will range from 0 to 1000 if the zone's population is
 * greater than 0.  Returns -1 otherwise.
 */
KND_INLINE int zone_yc(struct zone *zone)
{
	return zone->pop > 0 ?
		(int)((zone->ysum / zone->pop - zone->ymin) * 1000 / (zone->ymax - zone->ymin)) :
		-1;
}

/*
 * Calculates the proportional Z-axis center of gravity for the given zone.
 * The return value will range from 0 to 1000 if the zone's population is
 * greater than 0.  Returns -1 otherwise.
 */
KND_INLINE int zone_zc(struct zone *zone)
{
	return zone->pop > 0 ?
		(int)((zone->zsum / zone->pop - zone->zmin) * 1000 / (zone->zmax - zone->zmin)) :
		-1;
}


/***** kndsrv.c *****/

/*
 * Creates a server for the given knd context.  Call kndsrv_run() to run the
 * server's event loop.  kndsrv_run() should be called shortly after the server
 * is created, as the socket begins waiting for connections immediately.  Call
 * kndsrv_stop() in another thread, an event handler, or a signal handler to
 * stop the running server.  Pass 0 for port to use the default port.  Returns
 * NULL on error.
 */
struct knd_server *kndsrv_create(struct knd_info *info, unsigned short port);

/*
 * Destroys the given server.  This should not be called while the server's
 * event loop is running.  Instead, call kndsrv_stop(), then call
 * kndsrv_destroy() when kndsrv_stop() returns.
 */
void kndsrv_destroy(struct knd_server *server);

/*
 * Starts the given server's event loop in a newly-created thread.  Returns 0
 * on success, -1 on error.
 */
int kndsrv_run(struct knd_server *server);

/*
 * Stops the given server's event loop.  This function waits for the server
 * event thread to exit.
 */
void kndsrv_stop(struct knd_server *server);

/*
 * Writes a depth wakeup instruction to the given server's wakeup pipe.  Call
 * this when a depth frame is received.
 */
void kndsrv_send_depth(struct knd_server *server);

/*
 * Writes a video wakeup instruction to the given server's wakeup pipe.  Call
 * this when a video frame is received.
 */
void kndsrv_send_video(struct knd_server *server);


/***** save.c *****/

/*
 * Initializes zone saving (makes sure the directory exists and is writable).
 * This should not be called while the given zone list is being accessed by
 * another thread.  Returns a pointer to a save_info struct on success, NULL on
 * error.
 */
struct save_info *init_save(struct knd_info *knd, struct zonelist *zones, const char *savedir, struct timespec *interval);

/*
 * Frees the given save information.  Call this *before* the associated zone
 * list is freed.
 */
void cleanup_save(struct save_info *info);

/*
 * Unconditionally saves the zone list associated with the given save info.
 * Returns 0 on success, -1 on error.
 */
int save_zones(struct save_info *info);

/*
 * Loads zone information, if it exists, from the directory pointed to by info.
 * Does not remove any existing zones from the zone list.  Returns number of
 * zones read on success, -1 on error.
 */
int load_zones(struct save_info *info);

/*
 * Saves the zone list associated with this save info if its save interval has
 * elapsed since the last save and the zone list's version has changed.  On the
 * off chance that approximately (2^32)-1 zone changes occur between calls to
 * this function, the zones will not be saved.  This function should not be
 * called while the zone list is locked.  Returns 1 if the zones were not
 * saved, 0 on successful save, -1 on error.
 */
int check_save(struct save_info *info);


#endif /* _KND_H_ */
