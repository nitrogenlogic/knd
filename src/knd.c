/*
 * knd.c - Depth camera daemon initialization code and main loop
 * Copyright (C)2013 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <sched.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <unistd.h>
#include <execinfo.h>
#include <time.h>
#include <sys/prctl.h>
#include <ucontext.h>
#include <sys/syscall.h>//XXX
#include <dlfcn.h>

#include "knd.h"

// Design goals/ideas:
// - Accept multiple TCP/IP connections (UNIX domain as well?)
// - Ability to listen on just localhost, a specific interface, or all
//   interfaces (implement localhost and all interfaces first)
// - Global set of zones that any incoming connection can request.
//   - Example: zones defined via web interface.  Web interface connects to knd,
//     specifies global zones.  Other clients connect and request status of
//     global zones.
// - Ability to retrieve raw depth data, depth background, depth image, and
//   color image
// - Save/load global zones to/from disk
// - Threads:
//   - Watchdog thread: periodically checks for camera event thread hangs.
//   - Main/event thread: runs event processing loop.
//   - Data processing thread: accepts buffers captured in event processing
//     loop, checks global zones, checks connection-specific zones.
//   - Server thread: runs libevent loop.  Possibly create an open source
//     library that simplifies creation of a simple socket server with libevent.
// - Modules (i.e. source files):
//   - Main module (this file): main(), signal handler(s), watchdog callback
//   - Watchdog implementation (watchdog.c)
//   - Depth and video data processing (vidproc.c)
//   - Zone processing (zone.c)
//   - Server (kndsrv.c)
//   - Zone saving (save.c)
// - Performance optimizations:
//   - Store zones in a space-partitioning tree to reduce operations per pixel.
//   - Store zones in a zone bitmap, which may be per-pixel, or may be a much
//     lower resolution (a very low-resolution zone bitmap would be similar to
//     the zone partitioning tree mentioned above).  If more than [wordsize]
//     zones are present, the MSB of the zone bitfield may indicate that all
//     zones numbered wordsize or higher must be checked (in other words, the
//     MSB will be set if one or more higher-numbered zones are present at that
//     location in the zone map).
//
//     Alternatively, if more than [wordsize] zones are present, each bit in
//     the bitmap corresponds to the [wordsize]th entry in the zone table.  A
//     loop that increments by [wordsize] on each iteration could check all
//     zones that match a particular bit.  Or, the loop increment could be one
//     plus the number of zero bits between the current 1 bit and the next 1
//     bit in the bitmap.
//   - Zone spans: split the scene into (probably rectangular) spans of
//     overlapping zones.  Each span is an area in screen space in which all
//     pixels are covered by the same set of zones.  A span may also include
//     depth range information, so out-of-range depth samples can be skipped
//     without having to check each zone in the span.  Approach 1: the depth
//     image is still scanned across in horizontal scan lines.  The inner loop
//     is segmented by span; within each span's partial scan line, only the
//     span's corresponding zones will be checked for a depth match.  Approach
//     2: spans are sorted in a way to optimize cache locality (e.g. a
//     space-filling function), spans are scanned in order, and only those
//     pixels covered by a span are scanned.
//   - Use pixel-space bounding box before doing full world-space comparison?
//     If supporting rotated rectangles, the following may be useful:
//     http://twanvl.nl/blog/haskell/finding-rectangles
//   - If/when non-perpendicular zones are supported, use pixel and world
//     bounding box comparison before doing full comparison.  Using a Largest
//     Empty Rectangle to avoid detailed shape comparisons within the center of
//     a zone may help as well.  For simple non-perpendicular rectangular
//     zones, calculation of the LER should be fairly straightforward (some
//     kind of interpolation along the edges of the rectangle as it is rotated
//     relative to view space).
//   - Optional frame and/or pixel decimation settings.
//   - Selectable nice level (default to positive niceness).

static void watchdog_callback(void *data, struct timespec *interval)
{
	struct knd_info *info = data;

	ERROR_OUT("Timed out: at least %ld.%09lds since last update.\n", (long)interval->tv_sec, (long)interval->tv_nsec);

	if(!info->stop) {
		pthread_kill(info->thread_ctx->main_thread, SIGUSR2);
		info->stop = 1;
	} else {
		raise(SIGTERM);
	}

	// Wait another full watchdog interval before sending SIGTERM
	kick_watchdog(info->wd);
}

static void depth_callback(uint8_t *buffer, void *data)
{
	struct knd_info *info = data;
	struct timespec this_time;

	kick_watchdog(info->wd);

	update_zonelist_depth(info->zones, buffer);

	// Calculate frame rate (TODO: do this in another thread so the fps
	// goes to 0 when data stops coming)
	info->frames++;
	clock_gettime(CLOCK_MONOTONIC, &this_time);
	if(this_time.tv_sec > info->next_time.tv_sec ||
			(this_time.tv_sec == info->next_time.tv_sec &&
			 this_time.tv_nsec > info->next_time.tv_nsec)) {
		info->fps = info->frames * 100 /
			((this_time.tv_sec - info->last_time.tv_sec) * 100 +
			 (this_time.tv_nsec - info->last_time.tv_nsec) / 10000000);
		info->last_time = this_time;
		info->next_time = info->last_time;
		info->next_time.tv_nsec += 200000000;
		if(info->next_time.tv_nsec >= 1000000000) {
			info->next_time.tv_nsec -= 1000000000;
			info->next_time.tv_sec++;
		}
		info->frames = 0;
	}

	// Tell the server to process subscriptions
	kndsrv_send_depth(info->srv);
}

static void video_callback(uint8_t *buffer, void *data)
{
	struct knd_info *info = data;
	update_zonelist_video(info->zones, buffer);
	kndsrv_send_video(info->srv);
}

/*
 * Signal handler and its data.
 */
static struct knd_info *sigdata;
static void intr(int signum)
{
	sigdata->stop = 1;

	nl_ptmf("Exiting due to signal %d (%s).\n", signum, strsignal(signum));

	// Exit if this signal is received again
	signal(signum, exit);
}

static void crash(int signum, siginfo_t *info, void *ctx)
{
	nl_print_signal(stderr, signum == SIGUSR2 ? "Watchdog sent" : "Crashing due to", info);
	nl_print_context(stderr, (ucontext_t *)ctx);

	NL_PRINT_TRACE(stderr);

	pthread_t self = pthread_self();
	if(pthread_equal(sigdata->thread_ctx->main_thread, self)) {
		nl_fptmf(stderr, "Main thread crash.\n");
	} else {
		nl_fptmf(stderr, "Helper thread crash.\n");
	}

	if(!sigdata->crashing) {
		nl_fptmf(stderr, "First handler to receive crash.  Notifying other threads.\n");
		sigdata->crashing = 1;

		nl_signal_threads(sigdata->thread_ctx, signum);

		usleep(250000);
		if(signum != SIGUSR2) {
			exit(-1);
		}
	} else {
		nl_fptmf(stderr, "Already crashing.  Nothing more to do.\n");
		if(signum != SIGUSR2) {
			pthread_exit(NULL);
		}
	}
}

int main(int argc, char *argv[])
{
	struct knd_info *info;
	const char *savedir = NULL;
	int savetime = 2;
	float init_timeout = 7, run_timeout = 0.75;

	if(argc == 2 && !strcmp(argv[1], "--help")) {
		printf("Usage:\n");
		printf("\t%s\n", argv[0]);
		printf("\nEnvironment variables:\n");
		printf("\tKND_INITTIMEOUT - Initialization timeout (defaults to 7 seconds)\n");
		printf("\tKND_RUNTIMEOUT - Runtime timeout (defaults to 0.75 seconds)\n");
		printf("\tKND_SAVEDIR - Sets data location (no default; zones are not saved without this variable)\n");
		printf("\nExample:\n");
		printf("\tKND_SAVEDIR=/var/tmp %s\n", argv[0]);
		exit(0);
	}

	nl_set_threadname("main_thread");

	if(getenv("KND_INITTIMEOUT") != NULL) {
		init_timeout = atof(getenv("KND_INITTIMEOUT"));
		nl_ptmf("Setting init timeout to %f\n", init_timeout);
	}

	if(getenv("KND_RUNTIMEOUT") != NULL) {
		run_timeout = atof(getenv("KND_RUNTIMEOUT"));
		nl_ptmf("Setting run timeout to %f\n", run_timeout);
	}

	if(getenv("KND_SAVEDIR") != NULL) {
		savedir = getenv("KND_SAVEDIR");
		nl_ptmf("Setting save location to '%s'\n", savedir);
	}

	// TODO: KND_SAVETIME -- save interval in seconds

	init_lut();

	sigdata = info = calloc(1, sizeof(struct knd_info));
	if(info == NULL) {
		ERRNO_OUT("Error allocating memory for server information.\n");
		return -1;
	}

	info->thread_ctx = nl_create_thread_context();
	if(info->thread_ctx == NULL) {
		ERRNO_OUT("Error creating threading context");
		return -1;
	}

	info->zones = create_zonelist(2, 2);
	if(info->zones == NULL) {
		ERROR_OUT("Error creating global zone list.\n");
		return -1;
	}

	if(savedir != NULL) {
		nl_ptmf("Initializing zone persistence.\n");
		info->save = init_save(info, info->zones, savedir, &(struct timespec){.tv_sec = savetime, .tv_nsec = 0});
		if(info->save == NULL) {
			ERROR_OUT("Error initializing zone saving.\n");
			return -1;
		}
	}

	struct sigaction crash_action = {
		.sa_sigaction = crash,
		.sa_flags = SA_SIGINFO,
	};
	sigemptyset(&crash_action.sa_mask);
	if(signal(SIGTERM, intr) == SIG_ERR ||
			signal(SIGINT, intr) == SIG_ERR ||
			sigaction(SIGFPE, &crash_action, NULL) ||
			sigaction(SIGILL, &crash_action, NULL) ||
			sigaction(SIGBUS, &crash_action, NULL) ||
			sigaction(SIGSEGV, &crash_action, NULL) ||
			sigaction(SIGUSR2, &crash_action, NULL)) {
		ERROR_OUT("Error setting termination signal handlers.\n");
		free(info);
		return -1;
	}

	nl_ptmf("Creating server.\n");
	info->srv = kndsrv_create(info, 0);
	if(info->srv == NULL) {
		ERROR_OUT("Error creating server.");
		free(info);
		return -1;
	}

	nl_ptmf("Creating watchdog.\n");
	info->wd = create_watchdog(
			info,
			&(struct timespec){ .tv_sec = 0, .tv_nsec = 255000000 },
			&(struct timespec){ .tv_sec = (int)init_timeout,
			.tv_nsec = (int)((init_timeout - (int)init_timeout) * 1000000000) },
			info,
			watchdog_callback
			);
	if(info->wd == NULL) {
		ERROR_OUT("Error creating watchdog.\n");
		free(info);
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &info->last_time);
	info->next_time = info->last_time;
	info->next_time.tv_nsec += 500000000;
	if(info->next_time.tv_nsec >= 1000000000) {
		info->next_time.tv_nsec -= 1000000000;
		info->next_time.tv_sec++;
	}

	// TODO: Tilt camera up and down a few degrees to re-align motor

	nl_ptmf("Starting video processing.\n");
	info->vid = init_vidproc(info, 0, depth_callback, info, video_callback, info);
	if(info->vid == NULL) {
		ERROR_OUT("Error initializing video processing.\n");
		destroy_watchdog(info->wd);
		free(info);
		return -1;
	}

	if(savedir != NULL) {
		nl_ptmf("Loading saved zones.\n");
		int zone_count = load_zones(info->save);
		if(zone_count < 0) {
			ERROR_OUT("Error loading saved zones.\n");
		} else {
			nl_ptmf("Loaded %d zone(s).\n", zone_count);
		}
	}

	nl_ptmf("Starting server.\n");
	if(kndsrv_run(info->srv)) {
		ERROR_OUT("Error starting server.\n");
		cleanup_vidproc(info->vid);
		destroy_watchdog(info->wd);
		destroy_zonelist(info->zones);
		free(info);
		return -1;
	}

	set_watchdog_timeout(
			info->wd,
			&(struct timespec){ .tv_sec = (int)run_timeout,
			.tv_nsec = (int)((run_timeout - (int)run_timeout) * 1000000000) });

	nl_ptmf("Starting event processing.\n");
	while(!info->stop) {
		if(vidproc_doevents(info->vid)) {
			break;
		}
	}

	nl_ptmf("Stopping server.\n");
	kndsrv_stop(info->srv);

	if(info->save != NULL) {
		nl_ptmf("Saving zones.\n");
		save_zones(info->save);
		cleanup_save(info->save);
	}

	nl_ptmf("Stopping video processing.\n");
	cleanup_vidproc(info->vid);

	nl_ptmf("Destroying server.\n");
	kndsrv_destroy(info->srv);

	nl_ptmf("Destroying watchdog.\n");
	destroy_watchdog(info->wd);

	nl_ptmf("Cleaning up.\n");
	destroy_zonelist(info->zones);
	nl_destroy_thread_context(info->thread_ctx);
	free(info);

	return 0;
}

