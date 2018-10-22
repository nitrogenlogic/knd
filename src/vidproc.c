/*
 * vidproc.c - Depth camera daemon video processing code and libfreenect callbacks
 * Copyright (C)2012 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdlib.h>
#include <semaphore.h>
#include <math.h>
#include <asm/byteorder.h>

#include <libusb-1.0/libusb.h>

#include "knd.h"

struct vidproc_info {
	struct knd_info *knd;

	uint32_t depth_timestamp; // Depth timestamp
	uint8_t *depth_buffer; // Depth buffer
	sem_t depth_full; // Posted by depth callback
	sem_t depth_empty; // Posted by depth thread
	pthread_mutex_t depth_in_use;
	int busy_count; // The number of times depth callback timed out on semaphore

	uint32_t last_depth;
	unsigned int depth_frames;
	void *depth_cb_data;
	vidproc_func depth_cb;

	uint32_t video_timestamp;
	uint8_t *video_buffer;
	sem_t video_full; // Posted by video callback
	sem_t video_empty; // Posted by video thread
	uint32_t video_requested:1;
	uint32_t video_started:1;
	pthread_mutex_t video_in_use;

	uint32_t last_video;
	unsigned int video_frames;
	void *video_cb_data;
	vidproc_func video_cb;

	libusb_context *camera_usb;
	libusb_context *motor_usb;
	freenect_context *camera_ctx;
	freenect_context *motor_ctx;
	freenect_device *camera_dev;
	freenect_device *motor_dev;
	freenect_led_options led;
	freenect_led_options last_led;

	// Video capture LED indicator tracking
	struct timespec end_depth;
	struct timespec end_video;

	int tilt;
	int last_tilt;

	unsigned int motor_missing:1;

	struct nl_thread *depth_thread;
	struct nl_thread *video_thread;

	pthread_mutex_t param_mutex;
	volatile unsigned int stop:1;
};

enum frame_type {
	DEPTH,
	VIDEO,
};


/*
 * Depth look-up table (translates depth sample into world-space millimeters).
 */
int depth_lut[2048];

/*
 * Surface area look-up table (translates depth sample into world-space surface
 * area of a pixel at that distance).
 */
float surface_lut[2048];

/*
 * Whether the look-up table has yet been initialized.
 */
static int lut_filled = 0;

/*
 * Returns the surface area of a single pixel at the given distance.  Works for
 * any unit (mm->mm^2, m->m^2, etc.).  Does not use the surface area look-up
 * table.
 */
float surface_area(float z)
{
	// 2.760888e-6 ~= (tan(28)/320)^2
	return z * z * 2.760888e-6f;
}

/*
 * Initializes the depth look-up table.
 */
void init_lut()
{
	float d;
	int i;

	if(lut_filled) {
		return;
	}

	for(i = 0; i < 2048; i++) {
		d = 1000.0f * 0.1236f * tanf(i / 2842.5f + 1.1863f);
		depth_lut[i] = (int)d;
		surface_lut[i] = surface_area(d);
	}

	lut_filled = 1;
}

/*
 * Wraps read access to the stop flag in a mutex to quiet Helgrind/DRD.
 */
static int get_stop(struct vidproc_info *info)
{
	int ret, val;

	ret = pthread_mutex_lock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error locking vidproc param mutex: %s\n", strerror(ret));
	}
	val = info->stop;
	ret = pthread_mutex_unlock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error unlocking vidproc param mutex: %s\n", strerror(ret));
	}

	return val;
}

/*
 * Wraps write access to the stop flag in a mutex.
 */
static void set_stop(struct vidproc_info *info, int stop)
{
	int ret;

	ret = pthread_mutex_lock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error locking vidproc param mutex: %s\n", strerror(ret));
	}
	info->stop = !!stop;
	ret = pthread_mutex_unlock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error unlocking vidproc param mutex: %s\n", strerror(ret));
	}
}

// Resets the LED time for the given type of capture.  For VIDEO frames, the
// video_in_use mutex should be locked before calling.  For depth frames, the
// depth_in_use mutex should be locked before calling.
static void kick_led(struct vidproc_info *info, enum frame_type type)
{
	const struct timespec depth_hold = { .tv_sec = 2 };
	const struct timespec video_hold = { .tv_sec = 3 };

	switch(type) {
		case DEPTH:
			nl_clock_fromnow(CLOCK_MONOTONIC, &info->end_depth, depth_hold);
			break;

		case VIDEO:
			nl_clock_fromnow(CLOCK_MONOTONIC, &info->end_video, video_hold);
			break;
	}
}

// Sets the LED color based on timespecs set by kick_led() (the LED color is
// actually sent to libfreenect elsewhere).
static void update_led(struct vidproc_info *info)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	if(!NL_TIMESPEC_GTE(now, info->end_video)) {
		info->led = LED_RED;
	} else if(!NL_TIMESPEC_GTE(now, info->end_depth)) {
		info->led = LED_YELLOW;
	} else {
		info->led = LED_GREEN;
	}
}

static void *depth_thread(void *d)
{
	struct vidproc_info *info = d;
	int error_count = 0;
	int ret;

	nl_ptmf("Depth thread started.\n");

	while(!get_stop(info) && error_count < 3) {
		if(sem_wait(&info->depth_full)) {
			ERRNO_OUT("Error waiting for depth buffer to be full");
			error_count++;
			continue;
		}

		if(get_stop(info)) {
			// Stop before processing any data
			break;
		}

		ret = pthread_mutex_lock(&info->depth_in_use);
		if(ret) {
			ERROR_OUT("Error locking depth buffer mutex: %s\n", strerror(ret));
		}

		if(info->depth_frames == 1) {
			nl_ptmf("Received first depth frame.\n");
		}

		if(info->depth_cb != NULL) {
			info->depth_cb(info->depth_buffer, info->depth_cb_data);
		}

		update_led(info);

		if(sem_post(&info->depth_empty)) {
			ERRNO_OUT("Error notifying event thread depth is empty");
			error_count++;
		}

		ret = pthread_mutex_unlock(&info->depth_in_use);
		if(ret) {
			ERROR_OUT("Error unlocking depth buffer mutex: %s\n", strerror(ret));
		}
	}

	nl_ptmf("Depth thread exiting.\n");

	return NULL;
}

static void *video_thread(void *d)
{
	struct vidproc_info *info = d;
	int error_count = 0;
	int ret;

	nl_ptmf("Video thread started.\n");

	while(!get_stop(info) && error_count < 3) {
		if(sem_wait(&info->video_full)) {
			ERRNO_OUT("Error waiting for video buffer to be full");
			error_count++;
			continue;
		}

		if(get_stop(info)) {
			// Stop before processing any data
			break;
		}

		ret = pthread_mutex_lock(&info->video_in_use);
		if(ret) {
			ERROR_OUT("Error locking video buffer mutex: %s\n", strerror(ret));
		}

		if(info->video_frames == 1) {
			nl_ptmf("Received first video frame.\n");
		}

		if(info->video_cb != NULL) {
			info->video_cb(info->video_buffer, info->video_cb_data);
		}

		if(sem_post(&info->video_empty)) {
			ERRNO_OUT("Error notifying event thread video is empty");
			error_count++;
		}

		ret = pthread_mutex_unlock(&info->video_in_use);
		if(ret) {
			ERROR_OUT("Error unlocking video buffer mutex: %s\n", strerror(ret));
		}
	}

	nl_ptmf("Video thread exiting.\n");

	return NULL;
}

static void depth_callback(freenect_device *dev, void *depthbuf, uint32_t timestamp)
{
	struct vidproc_info *info = freenect_get_user(dev);
	int ret;

	if(sem_timedwait(&info->depth_empty, &(struct timespec){.tv_nsec = 1000000})) {
		if(errno == ETIMEDOUT) {
			// TODO: Add a way of getting the busy count and comparing it to depth_frames + busy_count
			// (e.g. show processed fps vs. received fps)
			info->busy_count++;
		} else {
			ERRNO_OUT("Error waiting for depth buffer to be empty");
		}
		return;
	}

	if((ret = pthread_mutex_lock(&info->depth_in_use))) {
		ERROR_OUT("Error waiting for exclusive access to depth buffer: %s\n", strerror(ret));
		sem_post(&info->depth_empty);
		return;
	}

	memcpy(info->depth_buffer, depthbuf, KND_DEPTH_SIZE);
	info->last_depth = info->depth_timestamp;
	info->depth_timestamp = timestamp;
	info->depth_frames++;

	if(sem_post(&info->depth_full)) {
		ERRNO_OUT("Error posting depth to processing thread");
	}

	if((ret = pthread_mutex_unlock(&info->depth_in_use))) {
		ERROR_OUT("Error unlocking depth buffer: %s\n", strerror(ret));
	}
}

static void video_callback(freenect_device *dev, void *videobuf, uint32_t timestamp)
{
	struct vidproc_info *info = freenect_get_user(dev);
	int ret;

	if(sem_wait(&info->video_empty)) {
		ERRNO_OUT("Error waiting for video buffer to be empty");
		return;
	}

	if((ret = pthread_mutex_lock(&info->video_in_use))) {
		ERROR_OUT("Error waiting for exclusive access to video buffer: %s\n", strerror(ret));
		sem_post(&info->video_empty);
		return;
	}

	memcpy(info->video_buffer, videobuf, KND_VIDEO_SIZE);
	info->last_video = info->video_timestamp;
	info->video_timestamp = timestamp;
	info->video_frames++;
	info->video_requested = 0;

	if(sem_post(&info->video_full)) {
		ERRNO_OUT("Error posting video to processing thread");
	}

	if((ret = pthread_mutex_unlock(&info->video_in_use))) {
		ERROR_OUT("Error unlocking video buffer: %s\n", strerror(ret));
	}
}

// libfreenect logging callback
static void log_callback(freenect_context *ctx, freenect_loglevel level, const char *msg)
{
	static const char * const levels[] = {
		"fatal error",
		"error",
		"warning",
		"notice",
		"info",
		"debug",
		"spew",
		"flood",
	};

	nl_ptmf("Camera %s: %s", levels[level], msg);
}

/*
 * Initializes libfreenect and opens the devindex-th camera.  If depth_cb
 * and/or video_cb are not NULL, then they will be called for every frame
 * received and processed.  Callbacks are called from the vidproc thread after
 * vidproc's own processing is complete, so use appropriate locking and avoid
 * spending too much time in callbacks.
 */
struct vidproc_info *init_vidproc(struct knd_info *knd, int devindex, vidproc_func depth_cb, void *depth_cb_data, vidproc_func video_cb, void *video_cb_data)
{
	struct vidproc_info *info;
	pthread_mutexattr_t mutex_attr;
	int devcount;
	int ret;

	if((ret = pthread_mutexattr_init(&mutex_attr)) ||
			(ret = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK))) {
		ERROR_OUT("Error initializing mutex attributes: %s\n", strerror(ret));
		return NULL;
	}

	info = calloc(1, sizeof(struct vidproc_info));
	if(info == NULL) {
		ERRNO_OUT("Error allocating memory for video processing information");
		goto error1;
	}

	if((ret = pthread_mutex_init(&info->param_mutex, &mutex_attr))) {
		ERROR_OUT("Error creating run/stop mutex: %s\n", strerror(ret));
		goto error2;
	}

	if((ret = pthread_mutex_init(&info->depth_in_use, &mutex_attr))) {
		ERROR_OUT("Error creating buffer mutex: %s\n", strerror(ret));
		goto error3;
	}

	if(sem_init(&info->depth_full, 0, 0)) {
		ERRNO_OUT("Error creating depth buffer full semaphore");
		goto error4;
	}
	if(sem_init(&info->depth_empty, 0, 1)) { // Initial value 1 to allow callback to start
		ERRNO_OUT("Error creating depth buffer empty semaphore");
		goto error5;
	}

	if(sem_init(&info->video_full, 0, 0)) {
		ERRNO_OUT("Error creating video buffer full semaphore");
		goto error6;
	}
	if(sem_init(&info->video_empty, 0, 1)) { // Initial value 1 to allow callback to start
		ERRNO_OUT("Error creating video buffer empty semaphore");
		goto error7;
	}

	info->depth_buffer = malloc(FREENECT_DEPTH_11BIT_PACKED_SIZE);
	if(info->depth_buffer == NULL) {
		ERRNO_OUT("Error allocating depth image buffer");
		goto error;
	}

	info->video_buffer = malloc(KND_VIDEO_SIZE);
	if(info->video_buffer == NULL) {
		ERRNO_OUT("Error allocating video image buffer");
		goto error;
	}

	if((ret = libusb_init(&info->camera_usb)) != 0) {
		ERROR_OUT("Error initializing libusb camera context: %d.\n", ret);
		goto error;
	}
	if((ret = libusb_init(&info->motor_usb)) != 0) {
		ERROR_OUT("Error initializing libusb motor context: %d.\n", ret);
		goto error;
	}

	if(freenect_init(&info->motor_ctx, info->motor_usb) < 0) {
		ERROR_OUT("Error initializing libfreenect motor context.\n");
		goto error;
	}
	if(freenect_init(&info->camera_ctx, info->camera_usb) < 0) {
		ERROR_OUT("Error initializing libfreenect camera context.\n");
		goto error;
	}

	if(getenv("KND_LOG_LEVEL") != NULL) {
		freenect_set_log_level(info->motor_ctx, CLAMP(FREENECT_LOG_FATAL, FREENECT_LOG_FLOOD, atoi(getenv("KND_LOG_LEVEL"))));
		freenect_set_log_level(info->camera_ctx, CLAMP(FREENECT_LOG_FATAL, FREENECT_LOG_FLOOD, atoi(getenv("KND_LOG_LEVEL"))));
	} else {
		freenect_set_log_level(info->motor_ctx, FREENECT_LOG_ERROR);
		freenect_set_log_level(info->camera_ctx, FREENECT_LOG_ERROR);
	}
	freenect_set_log_callback(info->motor_ctx, log_callback);
	freenect_set_log_callback(info->camera_ctx, log_callback);

	freenect_select_subdevices(info->motor_ctx, FREENECT_DEVICE_MOTOR);
	freenect_select_subdevices(info->camera_ctx, FREENECT_DEVICE_CAMERA);

	devcount = freenect_num_devices(info->camera_ctx);
	if(devcount == 0) {
		ERROR_OUT("No depth cameras were found.\n");
		goto error;
	}
	if(devcount <= devindex) {
		ERROR_OUT("Requested depth camera %d (zero-indexed) does not exist (there are %d total).\n",
				devindex, devcount);
		goto error;
	}

	init_lut();

	// TODO: Use USB IDs to distinguish between K4Xbox old, K4Xbox new, and K4W
	if(freenect_open_device(info->motor_ctx, &info->motor_dev, devindex) < 0) {
		INFO_OUT("Opening motor %d (zero-indexed) failed.  Trying again.\n", devindex);
		usleep(500000);
		if(freenect_open_device(info->motor_ctx, &info->motor_dev, devindex) < 0) {
			INFO_OUT("Opening motor failed.  Operating without tilt and LED support.\n");
			info->motor_missing = 1;
		}
	}
	if(!info->motor_missing) {
		freenect_set_user(info->motor_dev, info);
	}

	if(freenect_open_device(info->camera_ctx, &info->camera_dev, devindex) < 0) {
		INFO_OUT("Opening camera %d (zero-indexed) failed.  Trying again.\n", devindex);
		usleep(500000);
		if(freenect_open_device(info->camera_ctx, &info->camera_dev, devindex) < 0) {
			ERROR_OUT("Error opening depth camera %d (zero-indexed).\n", devindex);
			goto error;
		}
	}

	freenect_set_user(info->camera_dev, info);
	freenect_set_depth_callback(info->camera_dev, depth_callback);
	if(freenect_set_depth_mode(info->camera_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT_PACKED))) {
		ERROR_OUT("Error setting depth resolution and image format.\n");
		goto error;
	}
	freenect_set_video_callback(info->camera_dev, video_callback);
	// TODO: Other video formats
	if(freenect_set_video_mode(info->camera_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, KND_VIDEO_FORMAT))) {
		ERROR_OUT("Error setting video resolution and image format.\n");
		goto error;
	}

	info->knd = knd;
	info->led = LED_GREEN;
	info->depth_cb = depth_cb;
	info->depth_cb_data = depth_cb_data;
	info->video_cb = video_cb;
	info->video_cb_data = video_cb_data;

	if(!info->motor_missing) {
		info->tilt = freenect_get_tilt_degs(freenect_get_tilt_state(info->motor_dev));
		info->last_tilt = info->tilt;
	}

	if(freenect_start_depth(info->camera_dev)) {
		ERROR_OUT("Error starting depth processing.\n");
		goto error;
	}

	ret = nl_create_thread(info->knd->thread_ctx, NULL, depth_thread, info, "depth_thread", &info->depth_thread);
	if(ret) {
		ERROR_OUT("Error creating depth processing thread: %s.\n", strerror(ret));
		goto error;
	}

	ret = nl_create_thread(info->knd->thread_ctx, NULL, video_thread, info, "video_thread", &info->video_thread);
	if(ret) {
		ERROR_OUT("Error creating video processing thread: %s.\n", strerror(ret));
		goto error;
	}

	pthread_mutexattr_destroy(&mutex_attr);
	return info;

	// An experiment in alternative exception-like error-handling methods
error:
	cleanup_vidproc(info);
	goto error1;

error7:
	sem_destroy(&info->video_full);
error6:
	sem_destroy(&info->depth_empty);
error5:
	sem_destroy(&info->depth_full);
error4:
	pthread_mutex_destroy(&info->depth_in_use);
error3:
	pthread_mutex_destroy(&info->param_mutex);
error2:
	free(info);
error1:
	pthread_mutexattr_destroy(&mutex_attr);

	return NULL;
}

/*
 * Stops video processing associated with the given info structure and frees
 * associated resources.  Should only be called after the libfreenect event
 * loop has exited. Ignores a null parameter.
 */
void cleanup_vidproc(struct vidproc_info *info)
{
	int ret;

	if(CHECK_NULL(info)) {
		return;
	}

	// Tell the depth and video processing thread to stop
	set_stop(info, 1);
	sem_post(&info->depth_full); // Kick it out of sem_wait()
	if(info->depth_thread) {
		ret = nl_join_thread(info->depth_thread, NULL);
		if(ret && ret != ESRCH) {
			ERROR_OUT("Error joining depth processing thread: %s\n", strerror(ret));
		}
	}

	sem_post(&info->video_full);
	if(info->video_thread) {
		ret = nl_join_thread(info->video_thread, NULL);
		if(ret && ret != ESRCH) {
			ERROR_OUT("Error joining video processing thread: %s\n", strerror(ret));
		}
	}

	if(info->motor_dev != NULL) {
		if(!info->motor_missing) {
			freenect_set_led(info->motor_dev, LED_OFF);
		}
		freenect_close_device(info->motor_dev);
	}
	if(info->motor_ctx != NULL) {
		freenect_shutdown(info->motor_ctx);
	}

	if(info->camera_dev != NULL) {
		freenect_stop_depth(info->camera_dev);
		freenect_stop_video(info->camera_dev);
		freenect_close_device(info->camera_dev);
	}
	if(info->camera_ctx != NULL) {
		freenect_shutdown(info->camera_ctx);
	}

	if(info->motor_usb != NULL) {
		libusb_exit(info->motor_usb);
	}
	if(info->camera_usb != NULL) {
		libusb_exit(info->camera_usb);
	}

	if((ret = pthread_mutex_lock(&info->depth_in_use))) {
		ERROR_OUT("Error locking depth buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	if((ret = pthread_mutex_lock(&info->video_in_use))) {
		ERROR_OUT("Error locking video buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	if(info->depth_buffer != NULL) {
		free(info->depth_buffer);
	}

	if(info->video_buffer != NULL) {
		free(info->video_buffer);
	}

	sem_destroy(&info->depth_full);
	sem_destroy(&info->depth_empty);

	sem_destroy(&info->video_full);
	sem_destroy(&info->video_empty);

	if((ret = pthread_mutex_unlock(&info->depth_in_use))) {
		ERROR_OUT("Error unlocking depth buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	if((ret = pthread_mutex_unlock(&info->video_in_use))) {
		ERROR_OUT("Error unlocking video buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	if((ret = pthread_mutex_destroy(&info->depth_in_use))) {
		ERROR_OUT("Error destroying depth buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	if((ret = pthread_mutex_destroy(&info->video_in_use))) {
		ERROR_OUT("Error destroying video buffer mutex while cleaning up: %s\n", strerror(ret));
	}

	free(info);
}

/*
 * Runs one iteration of libfreenect event processing.  This function blocks
 * until there are events to process, so it should be called through its own
 * event loop.  Returns 0 on success, -1 if there was any error other than an
 * interrupted system call (which happens frequently under some code profiling
 * tools).
 */
int vidproc_doevents(struct vidproc_info *info)
{
	int ret;

	if(CHECK_NULL(info)) {
		return -1;
	}

	// Process USB events
	ret = freenect_process_events(info->camera_ctx);
	if(ret && ret != LIBUSB_ERROR_INTERRUPTED) {
		return -1;
	}

	ret = freenect_process_events_timeout(info->motor_ctx,
			&(struct timeval){.tv_sec = 0, .tv_usec = 0});
	if(ret && ret != LIBUSB_ERROR_INTERRUPTED) {
		return -1;
	}

	if(!info->motor_missing) {
		// Update LED state
		if(info->led != info->last_led) {
			freenect_set_led(info->motor_dev, info->led);
			info->last_led = info->led;
		}

		// Update tilt
		if(info->tilt != info->last_tilt) {
			freenect_set_tilt_degs(info->motor_dev, info->tilt);
			info->last_tilt = info->tilt;
		}
	}

	// Start/stop video as required
	if((ret = pthread_mutex_lock(&info->video_in_use))) {
		ERROR_OUT("Error locking video mutex: %s\n", strerror(ret));
	}
	if(info->video_requested && !info->video_started) {
		if(freenect_start_video(info->camera_dev)) {
			ERROR_OUT("Error starting video processing.\n");
		} else {
			info->video_started = 1;
		}
	} else if(info->video_started && !info->video_requested) {
		if(freenect_stop_video(info->camera_dev)) {
			ERROR_OUT("Error stopping video processing.\n");
		}
		info->video_started = 0;
	}
	if((ret = pthread_mutex_unlock(&info->video_in_use))) {
		ERROR_OUT("Error unlocking video mutex: %s\n", strerror(ret));
	}

	return 0;
}

/*
 * Locks the depth buffer and calls the given callback once with a pointer to
 * the depth buffer.  Also schedules an update of the camera's LED to indicate
 * video recording.  Returns 0 on success, -1 on error.
 */
int get_depth(struct vidproc_info *info, vidproc_func cb, void *cb_data)
{
	int ret;

	if(CHECK_NULL(info) || CHECK_NULL(cb)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&info->depth_in_use))) {
		ERROR_OUT("Error locking buffer mutex: %s\n", strerror(ret));
		return -1;
	}

	kick_led(info, DEPTH);

	cb(info->depth_buffer, cb_data);

	if((ret = pthread_mutex_unlock(&info->depth_in_use))) {
		ERROR_OUT("Error unlocking buffer mutex: %s\n", strerror(ret));
		// Return 0 since the callback was called
	}

	return 0;
}

/*
 * Starts video processing to grab a single frame of video (the video callback
 * will stop video processing after receiving a single frame).  Sets the video
 * requested flag, so that multiple calls to request_video() without calling
 * get_video() will start video processing only once.  Returns 0 on success, -1
 * on error.
 */
int request_video(struct vidproc_info *info)
{
	int pth_ret;
	int ret = 0;

	if(CHECK_NULL(info)) {
		return -1;
	}

	if((pth_ret = pthread_mutex_lock(&info->video_in_use))) {
		ERROR_OUT("Error locking video mutex: %s\n", strerror(pth_ret));
		return -1;
	}

	info->video_requested = 1;

	// TODO: Have watchdog check for excessive delay after video requested

	if((pth_ret = pthread_mutex_unlock(&info->video_in_use))) {
		ERROR_OUT("Error unlocking video mutex: %s\n", strerror(pth_ret));
		// No change to return value
	}

	return ret;
}

/*
 * Locks the video buffer and calls the given callback once with a pointer to
 * the video buffer.  Also schedules an update of the camera's LED to indicate
 * video recording.  Returns 0 on success, -1 on error.
 */
int get_video(struct vidproc_info *info, vidproc_func cb, void *cb_data)
{
	int ret;

	if(CHECK_NULL(info) || CHECK_NULL(cb)) {
		return -1;
	}

	if((ret = pthread_mutex_lock(&info->video_in_use))) {
		ERROR_OUT("Error locking video mutex: %s\n", strerror(ret));
		return -1;
	}

	kick_led(info, VIDEO);

	cb(info->video_buffer, cb_data);

	if((ret = pthread_mutex_unlock(&info->video_in_use))) {
		ERROR_OUT("Error unlocking video mutex: %s\n", strerror(ret));
		// Return 0 since the callback was called
	}

	return 0;
}

/*
 * Finds the closest entry in the depth look-up table to the given world-space
 * depth value in millimeters without going over.  Uses a binary search.
 */
int reverse_lut(int zw)
{
	int idx = 546; // Maximum Z value is 1092
	int off = 273;

	while(off > 0 && depth_lut[idx] != zw) {
		if(depth_lut[idx] > zw) {
			idx -= off;
		} else if(depth_lut[idx] < zw) {
			idx += off;
		}

		off >>= 1;
	}

	// Binary search isn't perfect due to truncation, so find the optimum value
	while(depth_lut[idx] > zw && idx > 0) {
		idx--;
	}
	while(depth_lut[idx + 1] < zw && idx <= PXZMAX) {
		idx++;
	}

	return idx;
}

/*
 * Returns the currently-requested motor tilt in degrees from horizontal.  The
 * motor's actual current position may be different.
 */
int get_tilt(struct vidproc_info *info)
{
	int ret, val;

	ret = pthread_mutex_lock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error locking vidproc param mutex: %s\n", strerror(ret));
	}
	val = info->tilt;
	ret = pthread_mutex_unlock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error unlocking vidproc param mutex: %s\n", strerror(ret));
	}

	return val;
}

/*
 * Requests that the motor tilt the camera to the specified number of degrees
 * from horizontal.  Takes no action if the motor device was not opened.
 */
void set_tilt(struct vidproc_info *info, int tilt)
{
	int ret;

	if(info->motor_missing) {
		return;
	}

	tilt = CLAMP(-15, 15, tilt);

	ret = pthread_mutex_lock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error locking vidproc param mutex: %s\n", strerror(ret));
	}
	info->tilt = tilt;
	ret = pthread_mutex_unlock(&info->param_mutex);
	if(ret) {
		ERROR_OUT("Error unlocking vidproc param mutex: %s\n", strerror(ret));
	}
}
