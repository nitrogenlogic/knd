/*
 * watchdog.c - Depth camera daemon watchdog implementation.
 * Copyright (C)2012 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdlib.h>

#include "knd.h"

/*
 * Watchdog thread information
 */
struct knd_watchdog {
	struct knd_info *knd;

	struct timespec interval;	// Sampling interval
	struct timespec timeout;	// Timeout interval

	// Internal data
	struct timespec last_time;
	struct nl_thread *thread;
	volatile unsigned int run:1;	// Set to 1 to start loop, set to 0 to end loop
	volatile unsigned int stop:1;	// Set to 1 to abort before run gets set to 1
	pthread_mutex_t lock;

	// The data that was passed to create_watchdog().
	void *data;

	// Called on initial timeout, and every .interval afterward as long as
	// kick_watchdog() is not called for this watchdog.
	watchdog_func callback;
};

/*
 * Wraps read access to the stop flag in a mutex to quiet Helgrind/DRD.
 */
static int get_stop(struct knd_watchdog *wd)
{
	int ret, val;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	val = wd->stop;
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}

	return val;
}

/*
 * Wraps write access to the stop flag in a mutex.
 */
static void set_stop(struct knd_watchdog *wd, int stop)
{
	int ret;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	wd->stop = !!stop;
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}
}

/*
 * Wraps read access to the run flag in a mutex to quiet Helgrind/DRD.
 */
static int get_run(struct knd_watchdog *wd)
{
	int ret, val;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	val = wd->run;
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}

	return val;
}

/*
 * Wraps write access to the run flag in a mutex.
 */
static void set_run(struct knd_watchdog *wd, int run)
{
	int ret;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	wd->run = !!run;
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}
}

/*
 * Watchdog timer monitoring thread.
 */
static void *watchdog_thread(void *d)
{
	struct knd_watchdog *wd = d;
	struct timespec ts, to, next;
	int ret;

	nl_set_threadname("watchdog_thread");

	// FIXME: This starting gate should be implemented using POSIX
	// synchronization constructs, as was done in libaidsocket and in the
	// depth processing code.  The starting gate exists to prevent the
	// watchdog from running before create_watchdog() has finished
	// initializing everything.
	while(!get_run(wd) && !get_stop(wd)) {
		sched_yield();
	}

	clock_gettime(CLOCK_MONOTONIC, &next);
	while(get_run(wd)) {
		clock_gettime(CLOCK_MONOTONIC, &ts);

		next.tv_sec += wd->interval.tv_sec;
		next.tv_nsec += wd->interval.tv_nsec;
		if(next.tv_nsec >= 1000000000) {
			next.tv_sec++;
			next.tv_nsec -= 1000000000;
		}

		ret = pthread_mutex_lock(&wd->lock);
		if(ret) {
			ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
		}
		ts.tv_sec -= wd->last_time.tv_sec;
		ts.tv_nsec -= wd->last_time.tv_nsec;
		to = wd->timeout;
		ret = pthread_mutex_unlock(&wd->lock);
		if(ret) {
			ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
		}

		if(ts.tv_nsec < 0) {
			ts.tv_sec--;
			ts.tv_nsec += 1000000000;
		}

		if(ts.tv_sec > to.tv_sec ||
				(ts.tv_sec == to.tv_sec && ts.tv_nsec > to.tv_nsec)) {
			wd->callback(wd->data, &ts);
		}

		// Ignore EINTR (probably indicates it's time to exit)
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

	return NULL;
}

/*
 * Creates a new watchdog thread.  kick_watchdog() must be called every
 * timeout nsecs or faster after this function returns.  The watchdog will be
 * checked for a timeout every interval nsecs.  Returns the newly-created
 * watchdog on success, NULL on error.  Note that C99 compound literals may be
 * used for the interval and timeout parameters.
 */
struct knd_watchdog *create_watchdog(struct knd_info *knd, struct timespec *interval, struct timespec *timeout, void *data, watchdog_func callback)
{
	struct knd_watchdog *wd;
	pthread_mutexattr_t mutex_attr;
	int ret;

	if(CHECK_NULL(callback)) {
		return NULL;
	}
	if(interval->tv_sec < 0 || interval->tv_nsec < 0 || (interval->tv_sec == 0 && interval->tv_nsec == 0)) {
		ERROR_OUT("Interval is invalid.\n");
		return NULL;
	}
	if(timeout->tv_sec < 0 || timeout->tv_nsec < 0 || (timeout->tv_sec == 0 && timeout->tv_nsec == 0)) {
		ERROR_OUT("Timeout is invalid.\n");
		return NULL;
	}

	if((ret = pthread_mutexattr_init(&mutex_attr)) ||
			(ret = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK))) {
		ERROR_OUT("Error initializing mutex attributes: %s\n", strerror(ret));
		return NULL;
	}

	// Allocate memory
	wd = calloc(1, sizeof(struct knd_watchdog));
	if(wd == NULL) {
		ERRNO_OUT("Error allocating memory for watchdog information");
		pthread_mutexattr_destroy(&mutex_attr);
		return NULL;
	}

	// Initialize structure
	wd->knd = knd;
	wd->interval = *interval;
	wd->timeout = *timeout;
	wd->data = data;
	wd->callback = callback;
	clock_gettime(CLOCK_MONOTONIC, &wd->last_time);

	ret = pthread_mutex_init(&wd->lock, &mutex_attr);
	if(ret) {
		ERROR_OUT("Error creating watchdog mutex: %s\n", strerror(ret));
		free(wd);
		pthread_mutexattr_destroy(&mutex_attr);
	}

	// Create thread
	ret = nl_create_thread(wd->knd->thread_ctx, NULL, watchdog_thread, wd, "watchdog_thread", &wd->thread);
	if(ret) {
		ERROR_OUT("Error creating watchdog thread: %s\n", strerror(ret));
		pthread_mutex_destroy(&wd->lock);
		free(wd);
		pthread_mutexattr_destroy(&mutex_attr);
		return NULL;
	}
	set_run(wd, 1);

	pthread_mutexattr_destroy(&mutex_attr);
	return wd;
}

/*
 * Stops a running watchdog (waiting for it to exit) and frees its associated
 * resources.  Takes no action if wd is NULL.
 */
void destroy_watchdog(struct knd_watchdog *wd)
{
	int ret;

	if(CHECK_NULL(wd)) {
		return;
	}

	set_run(wd, 0);
	set_stop(wd, 1);
	ret = pthread_cancel(wd->thread->thread);
	if(ret && ret != ESRCH) {
		ERROR_OUT("Error canceling watchdog thread: %s\n", strerror(ret));
	}
	ret = nl_join_thread(wd->thread, NULL);
	if(ret && ret != ESRCH) {
		ERROR_OUT("Error joining watchdog thread: %s\n", strerror(ret));
	}

	ret = pthread_mutex_destroy(&wd->lock);
	if(ret) {
		ERROR_OUT("Error destroying watchdog mutex: %s\n", strerror(ret));
	}

	free(wd);
}

/*
 * Resets the given watchdog's timeout countdown.  Doesn't check for a NULL
 * watchdog.
 */
void kick_watchdog(struct knd_watchdog *wd)
{
	int ret;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	clock_gettime(CLOCK_MONOTONIC, &wd->last_time);
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}
}

/*
 * Sets the given watchdog's timeout in a thread-safe way.  Does not kick the
 * watchdog.
 */
void set_watchdog_timeout(struct knd_watchdog *wd, struct timespec *timeout)
{
	int ret;

	ret = pthread_mutex_lock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error locking watchdog mutex: %s\n", strerror(ret));
	}
	wd->timeout = *timeout;
	ret = pthread_mutex_unlock(&wd->lock);
	if(ret) {
		ERROR_OUT("Error unlocking watchdog mutex: %s\n", strerror(ret));
	}
}

