/*
 * kndsrv.c - TCP/IP server interface (derived from lsrv* in logic system)
 * Copyright (C)2012 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <stdint.h>
#define u_char uint8_t
#include <event.h>
#undef u_char

#include "knd.h"

// TODO: Split server from here and lsrv into command server and socket server
// libraries.

#define MAX_BUFFER_SIZE		131072
#define QUEUED_CONNECTIONS	8	// 2nd parameter to listen()
#define CLIENT_TIMEOUT		0	// No timeout
#define KND_PROTOCOL_VERSION	2	// Switched to millimeters in version 2

// Declares a function called [name]_func suitable for use as a command function
#define DECLARE_FUNC(name) \
	static void name##_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args);

#define ERROR_KNDSRV(client, ...) {\
	nl_fptmf(stderr, "\e[0;1m%s:%d: %s(): %s:%d: ", __FILE__, __LINE__,\
			__FUNCTION__, client->remote_addr, client->remote_port);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, "\e[0m\n");\
}

#define ERRNO_KNDSRV(client, ...) {\
	nl_fptmf(stderr, "\e[0;1m%s:%d: %s(): %s:%d: ", __FILE__, __LINE__,\
			__FUNCTION__, client->remote_addr, client->remote_port);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, ": %d (%s)\e[0m\n", errno, strerror(errno));\
}

#define INFO_KNDSRV(client, ...) {\
	nl_ptmf("%s:%d: %s(): %s:%d: ", __FILE__, __LINE__,\
			__FUNCTION__, client->remote_addr, client->remote_port);\
	printf(__VA_ARGS__);\
}
#ifdef DEBUG
#define DEBUG_KNDSRV(client, ...) INFO_KNDSRV(client, __VA_ARGS__)
#else /* DEBUG */
#define DEBUG_KNDSRV(client, ...)
#endif /* DEBUG */


/*
 * Depth camera server information.
 */
struct knd_server {
	struct knd_info *info;

	struct nl_thread *thread;

	struct knd_client *client_list; // First element is a placeholder list head
	int client_count;

	struct event_base *evloop;
	struct event *connect_event;
	int listenfd;

	struct event *wake_event;
	int wake_read; // Used only by the server's event loop
	int wake_write; // Used to wake or kill the server's event loop (write 'W' to wake, 'K' to kill)
};

/*
 * A single client connected to the server.
 */
struct knd_client {
	struct knd_server *server;

	struct knd_client *prev, *next;

	int fd;
	char *remote_addr;
	unsigned short remote_port;
	unsigned int shutdown_requested:1;
	unsigned int shutdown:1;

	// Subscription status flags
	unsigned int subglobal:1; // Whether the client is subscribed to global zones
	unsigned int subdepth:1;  // '' '' raw depth data
	unsigned int subbright:1; // '' '' zone brightness
	unsigned int subvideo:1;  // '' '' raw video data
	int depth_limit;	  // Number of depth frames to capture before unsubscribing (<= 0 to go forever)

	struct bufferevent *buf_event;
	struct evbuffer *buffer;

	char in_buffer[MAX_BUFFER_SIZE]; // TODO: Replace with an evbuffer
	size_t last_len; // amount of data left in buffer

	struct zonelist *zones;
};

struct knd_cmd;

/*
 * Command handler function pointer type definition.
 */
typedef void (*knd_cmd_func)(struct knd_client *client, struct knd_cmd *command, int argc, const char *args);

/*
 * Information about a single command supported by the server.
 */
struct knd_cmd {
	char *name;
	char *desc;
	knd_cmd_func func;
	// TODO: Change to space-separated arguments and add argc_min, argc_max, **argv
};


static void shutdown_client(struct knd_client *client)
{
	if(!client->shutdown && shutdown(client->fd, SHUT_RDWR)) {
		ERRNO_OUT("Error shutting down client connection on fd %d", client->fd);
	}
	client->shutdown = 1;
}

/*
 * Queues a client for shutdown once its write buffer is drained.  If the write
 * buffer is empty, the socket is shut down immediately (so queue any outgoing
 * writes before calling this function, then call flush_client() after).
 */
static void request_shutdown_client(struct knd_client *client)
{
	client->shutdown_requested = 1;
	if(EVBUFFER_LENGTH(client->buffer) == 0) {
		shutdown_client(client);
	}
}

/*
 * Queues data buffer for transmission, but doesn't actually send it to the
 * socket.
 */
void flush_client(struct knd_client *client)
{
	if(bufferevent_write_buffer(client->buf_event, client->buffer)) {
		ERROR_OUT("Error sending data to client on fd %d\n", client->fd);
	}
}

/*
 * Sends information about the given zone to the given client as a single-line
 * list of key-value pairs.  If full is nonzero, sends all zone attributes;
 * otherwise only sends occupied, pop, maxpop, and name.
 */
static void send_zone_info(struct knd_client *client, struct zone *zone, int full)
{
	int pop = MAX_NUM(1, zone->pop);

	if(full) {
		evbuffer_add_printf(client->buffer, "xmin=%d ymin=%d zmin=%d xmax=%d ymax=%d zmax=%d ",
				zone->xmin, zone->ymin, zone->zmin, zone->xmax, zone->ymax, zone->zmax);
		evbuffer_add_printf(client->buffer, "px_xmin=%d px_ymin=%d px_zmin=%d px_xmax=%d px_ymax=%d px_zmax=%d ",
				zone->px_xmin, zone->px_ymin, zone->px_zmin, zone->px_xmax, zone->px_ymax, zone->px_zmax);
		evbuffer_add_printf(client->buffer, "negate=%d param=%s on_level=%d off_level=%d on_delay=%d off_delay=%d ",
				zone->negate, param_ranges[zone->occupied_param].name,
				zone->rising_threshold, zone->falling_threshold,
				zone->rising_delay, zone->falling_delay);
	}

#ifdef DEBUG
	evbuffer_add_printf(client->buffer, "delay_count=%d ", zone->count);
#endif /* DEBUG */

	// sa= is an approximation of area that is accurate to 3-4 digits
	evbuffer_add_printf(client->buffer, "occupied=%u pop=%d maxpop=%d xc=%d yc=%d zc=%d sa=%d name=\"%s\"\n",
				zone->occupied ^ zone->negate, zone->pop, zone->maxpop,
				zone_xc(zone),
				zone_yc(zone),
				zone_zc(zone),
				zone->pop > 0 ? (int)(zone->pop * surface_area((float)zone->zsum / pop)) : 0,
				zone->name); // TODO: escape name
}


DECLARE_FUNC(bye);
DECLARE_FUNC(ver);
DECLARE_FUNC(help);
DECLARE_FUNC(addzone);
DECLARE_FUNC(setzone);
DECLARE_FUNC(rmzone);
DECLARE_FUNC(clear);
DECLARE_FUNC(zones);
DECLARE_FUNC(sub);
DECLARE_FUNC(unsub);
DECLARE_FUNC(getbright);
DECLARE_FUNC(getdepth);
DECLARE_FUNC(subdepth);
DECLARE_FUNC(unsubdepth);
DECLARE_FUNC(getvideo);
DECLARE_FUNC(tilt);
DECLARE_FUNC(fps);
DECLARE_FUNC(lut);
DECLARE_FUNC(sa);

#ifdef DEBUG
DECLARE_FUNC(die);
DECLARE_FUNC(segv);
#endif /* DEBUG */

static struct knd_cmd commands[] = {
	{ "bye", "Disconnects from the server.", bye_func },
	{ "ver", "Returns the server protocol version.", ver_func },
	{ "help", "Lists available commands.", help_func },
	{ "addzone", "Adds a new global zone (name, xmin, ymin, zmin, xmax, ymax, zmax).", addzone_func },
	{ "setzone", "Sets a zone's parameters (name, all, xmin, ymin, zmin, xmax, ymax, zmax or name, [attr], value).", setzone_func },
	{ "rmzone", "Removes a global zone (name).", rmzone_func },
	{ "clear", "Removes all global zones.", clear_func },
	{ "zones", "Lists all global zones.", zones_func },
	{ "sub", "Subscribe to global zone updates.", sub_func },
	{ "unsub", "Unsubscribe from global zone updates.", unsub_func },
	{ "getdepth", "Grabs a single 11-bit packed depth image (increments subscription count if already subscribed).", getdepth_func },
	{ "subdepth", "Subscribes to 11-bit packed depth data (count (optional, <=0 means forever)).", subdepth_func },
	{ "unsubdepth", "Unsubscribes from 11-bit packed depth data.", unsubdepth_func },
	{ "getbright", "Asynchronously returns the approximate brightness within each zone.", getbright_func },
	{ "getvideo", "Grabs a single video image.", getvideo_func },
	{ "tilt", "Sets or returns the camera tilt in degrees from horizontal.", tilt_func },
	{ "fps", "Returns the approximate frame rate (updated every 200ms).", fps_func },
	{ "lut", "Returns the depth look-up table, or looks up an entry in the table.", lut_func},
	{ "sa", "Returns the surface area look-up table, or looks up an entry in the table.", sa_func},

#ifdef DEBUG
	{ "die", "Shuts down the server.", die_func }, // TODO: Add a hidden flag so this command doesn't show up in help?
	{ "segv", "Causes a segmentation fault in the server thread (for testing crash handling).", segv_func },
	// TODO: Add a debugging command to trigger a watchdog timeout
#endif /* DEBUG */
};


static void bye_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	evbuffer_add_printf(client->buffer, "OK - Goodbye\n");
	request_shutdown_client(client);
}

static void ver_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	evbuffer_add_printf(client->buffer, "OK - Version %d\n", KND_PROTOCOL_VERSION);
}

static void help_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	unsigned int i;

	evbuffer_add_printf(client->buffer, "OK - %zu commands (app version " KND_VERSION ")\n", ARRAY_SIZE(commands));

	for(i = 0; i < ARRAY_SIZE(commands); i++) {
		evbuffer_add_printf(client->buffer, "%s - %s\n", commands[i].name, commands[i].desc);
	}
}

// Used by addzone_func() to send zone addition events.
static void process_addition(void *data, struct zone *zone)
{
	struct knd_client *client = data;
	struct knd_server *server = client->server;

	client = server->client_list->next;
	while(client != NULL) {
		if(client->subglobal) {
			evbuffer_add(client->buffer, "ADD - ", 6);
			send_zone_info(client, zone, 1);
		}
		client = client->next;
	}
}

static void addzone_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	char name[ZONE_NAME_LENGTH];
	int xmin, ymin, zmin, xmax, ymax, zmax;
	ptrdiff_t name_length;
	struct zone *zone;
	int ret;

	if(argc != 7) {
		evbuffer_add_printf(client->buffer, "ERR - Expected 7 parameters, got %d\n", argc);
		return;
	}

	// TODO: Change to space-separated quoted arguments

	name_length = strchr(args, ',') - args;
	if(name_length >= ZONE_NAME_LENGTH) {
		evbuffer_add_printf(client->buffer, "ERR - Name is too long (limit is %d bytes, got %zd)\n",
				ZONE_NAME_LENGTH - 1, (ssize_t)name_length);
		return;
	}

	// TODO: Use ZONE_NAME_LENGTH macro to sprintf a format string
	if((ret = sscanf(args, "%127[^,],%d,%d,%d,%d,%d,%d", name,
				&xmin, &ymin, &zmin,
				&xmax, &ymax, &zmax)) != 7) {
		evbuffer_add_printf(client->buffer,
				"ERR - Error parsing arguments (successfully parsed %d of 7)\n", ret);
		return;
	}

	// TODO: When using separate zone lists for connections, a global list
	// lock will be needed to prevent the zonelist list from being modified
	// during updates, and from being updated during modification.
	zone = add_zone(client->server->info->zones, name, xmin, ymin, zmin, xmax, ymax, zmax);
	if(zone == NULL) {
		evbuffer_add_printf(client->buffer, "ERR - Error adding zone \"%s\" to zone list.\n", name);
	} else {
		evbuffer_add_printf(client->buffer, "OK - Zone \"%s\" was added.\n", name);
		process_addition(client, zone);
	}
}

static void setzone_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	char name[ZONE_NAME_LENGTH], attr[16];
	struct zone *z;
	int xmin, ymin, zmin, xmax, ymax, zmax;
	ptrdiff_t len;
	int ret;

	if(argc != 3 && argc != 8) {
		evbuffer_add_printf(client->buffer, "ERR - Expected 3 or 8 parameters, got %d\n", argc);
		return;
	}

	len = strchr(args, ',') - args;
	if(len >= ZONE_NAME_LENGTH) {
		evbuffer_add_printf(client->buffer, "ERR - Name is too long (limit is %d bytes, got %zd)\n",
				ZONE_NAME_LENGTH - 1, (ssize_t)len);
		return;
	}

	// TODO: Use ZONE_NAME_LENGTH macro to sprintf a format string
	// TODO: Use strncmp and strcspn for unlimited attr length
	if((ret = sscanf(args, "%127[^,],%15[^,]", name, attr) != 2)) {
		evbuffer_add_printf(client->buffer,
				"ERR - Error parsing name and attr arguments (successfully parsed %d of 2)\n", ret);
		return;
	}

	// A race condition between find_zone and set_zone* is not possible
	// here because all calls to add/remove/modify zones happen in the
	// single libevent processing thread.
	z = find_zone(client->server->info->zones, name);
	if(z == NULL) {
		evbuffer_add_printf(client->buffer, "ERR - Zone \"%s\" does not exist.\n", name);
		return;
	}

	if(!strcmp(attr, "all")) {
		if(argc != 8) {
			evbuffer_add_printf(client->buffer, "ERR - The \"all\" attribute requires 8 parameters.\n");
			return;
		}

		if((ret = sscanf(args, "%*127[^,],%*15[^,],%d,%d,%d,%d,%d,%d",
						&xmin, &ymin, &zmin,
						&xmax, &ymax, &zmax)) != 6) {
			evbuffer_add_printf(client->buffer,
					"ERR - Error parsing value arguments (successfully parsed %d of 6)\n", ret);
			return;
		}

		if(set_zone(client->server->info->zones, z, xmin, ymin, zmin, xmax, ymax, zmax)) {
			evbuffer_add_printf(client->buffer, "ERR - Error updating zone \"%s\".\n", name);
		} else {
			evbuffer_add_printf(client->buffer, "OK - Zone \"%s\" was updated.\n", name);
		}
	} else {
		if(argc != 3) {
			evbuffer_add_printf(client->buffer, "ERR - Only the \"all\" attribute accepts 8 parameters.  Use 3.\n");
			return;
		}

		if(set_zone_attr(client->server->info->zones, z, attr, strrchr(args, ',') + 1)) {
			evbuffer_add_printf(client->buffer, "ERR - Error updating zone \"%s\".\n", name);
		} else {
			evbuffer_add_printf(client->buffer, "OK - Zone \"%s\" attribute \"%s\" was updated.\n", name, attr);
		}
	}
}

// Used by rmzone_func() and clear_func() to send zone removal events.
static void process_removal(void *data, struct zone *zone)
{
	struct knd_client *client = data;
	struct knd_server *server = client->server;

	client = server->client_list->next;
	while(client != NULL) {
		if(client->subglobal) {
			evbuffer_add_printf(client->buffer, "DEL - %s\n", zone->name);
		}
		client = client->next;
	}
}

static void rmzone_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	struct zone *zone;

	// TODO: Accept multiple arguments to remove multiple zones

	// Note: since kndsrv is single threaded and only kndsrv can trigger
	// zone removal, there is no possibility of a zone struct being freed
	// between find_zone() and remove_zone().

	zone = find_zone(client->server->info->zones, args);
	if(zone == NULL) {
		evbuffer_add_printf(client->buffer, "ERR - Zone \"%s\" not found.\n", args);
		return;
	}
	process_removal(client, zone); // Notify subscribed clients about zone removal
	if(remove_zone(client->server->info->zones, zone)) {
		evbuffer_add_printf(client->buffer, "ERR - Error removing zone.\n");
		return;
	}

	evbuffer_add_printf(client->buffer, "OK - Zone \"%s\" was removed.\n", args);
}

static void clear_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	iterate_zonelist(client->server->info->zones, process_removal, client);

	clear_zonelist(client->server->info->zones);
	evbuffer_add_printf(client->buffer, "OK - All zones were removed.\n");
}

// Used by zones_func to iterate over the list of zones
static void zones_callback(void *data, struct zone *zone)
{
	struct knd_client *client = data;
	send_zone_info(client, zone, 1);
}

static void zones_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	char *name;
	unsigned int version;
	int idx = -1;

	// FIXME: zone count could change between the following lines.  Use a
	// count generated by zones_callback.
	name = peak_zone(client->server->info->zones, &idx, NULL, NULL);
	version = get_zonelist_version(client->server->info->zones);
	evbuffer_add_printf(client->buffer, "OK - %d zones - Version %u, %d occupied, peak zone is %d \"%s\"\n",
			zone_count(client->server->info->zones), version,
			occupied_count(client->server->info->zones),
			idx, name ? name : "[none]");
	iterate_zonelist(client->server->info->zones, zones_callback, client);

	if(name) {
		free(name);
	}
}

// Used by sub_func to send the initial subscription values
static void sub_func_callback(void *data, struct zone *zone)
{
	struct knd_client *client = data;
	evbuffer_add(client->buffer, "SUB - ", 6);
	send_zone_info(client, zone, 1);
}

static void sub_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	client->subglobal = 1;
	evbuffer_add_printf(client->buffer, "OK - Subscribed to global zone updates\n");
	iterate_zonelist(client->server->info->zones, sub_func_callback, client);
}

static void unsub_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	client->subglobal = 0;
	evbuffer_add_printf(client->buffer, "OK - Unsubscribed from global zone updates\n");
}

static void getdepth_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	if(client->subdepth) {
		if(client->depth_limit <= 0) {
			evbuffer_add_printf(client->buffer, "ERR - Already subscribed to depth data\n");
		} else {
			client->depth_limit++;
			evbuffer_add_printf(client->buffer, "OK - Incremented depth subscription count to %d\n", client->depth_limit);
		}
	} else {
		client->depth_limit = 1;
		client->subdepth = 1;
		evbuffer_add_printf(client->buffer, "OK - Requested a single depth frame for delivery as a DEPTH message\n");
	}
}

static void subdepth_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	int count = -1;

	if(argc > 1) {
		evbuffer_add_printf(client->buffer, "ERR - Too many arguments (expected 0 or 1)\n");
		return;
	}
	if(argc == 1) {
		count = MAX_NUM(atoi(args), -1);
	}

	client->depth_limit = count;
	client->subdepth = 1;

	if(count > 0) {
		evbuffer_add_printf(client->buffer,
				"OK - %d depth frame(s) will be delivered as DEPTH messages\n",
				count);
	} else {
		evbuffer_add_printf(client->buffer,
				"OK - depth frames will be delivered as DEPTH messages until unsubscribed\n");
	}
}

static void unsubdepth_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	if(!client->subdepth) {
		evbuffer_add_printf(client->buffer, "ERR - Not subscribed to depth data\n");
	} else {
		client->subdepth = 0;
		client->depth_limit = -1;
		evbuffer_add_printf(client->buffer, "OK - Unsubscribed from depth data\n");
	}
}

static void getvideo_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	client->subvideo = 1;
	if(request_video(client->server->info->vid)) {
		evbuffer_add_printf(client->buffer, "ERR - Error requesting video from the camera\n");
	} else {
		evbuffer_add_printf(client->buffer, "OK - Requested delivery of a video frame\n");
	}
}

static void getbright_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	client->subbright = 1;
	if(request_video(client->server->info->vid)) {
		evbuffer_add_printf(client->buffer, "ERR - Error requesting video from the camera\n");
	} else {
		evbuffer_add_printf(client->buffer, "OK - Requested brightness for each zone\n");
	}
}

static void tilt_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	if(argc > 1) {
		evbuffer_add_printf(client->buffer, "ERR - Too many arguments (expected 0 or 1)\n");
		return;
	}

	if(argc == 1) {
		int tilt = CLAMP(-15, 15, atoi(args));
		set_tilt(client->server->info->vid, tilt);
		evbuffer_add_printf(client->buffer, "OK - Requested tilt of %d degrees\n", tilt);
	} else {
		evbuffer_add_printf(client->buffer, "OK - Current tilt is %d degrees\n", get_tilt(client->server->info->vid));
	}
}

static void fps_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	evbuffer_add_printf(client->buffer, "OK - %d fps\n", client->server->info->fps);
}

static void lut_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	int i;

	if(argc > 1) {
		evbuffer_add_printf(client->buffer, "ERR - Too many arguments (expected 0 or 1)\n");
		return;
	}

	if(argc == 1) {
		int d = atoi(args);

		if(d < 0 || d >= 2048) {
			evbuffer_add_printf(client->buffer, "ERR - Raw distance value %d is out of range (0-2047).\n", d);
			return;
		}

		evbuffer_add_printf(client->buffer, "OK - %d -> %dmm.\n", d, depth_lut[d]);
	} else {
		evbuffer_add_printf(client->buffer, "OK - 2048 lines follow\n");
		for(i = 0; i < 2048; i++) {
			evbuffer_add_printf(client->buffer, "%d\n", depth_lut[i]);
		}
	}
}

static void sa_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	int i;

	if(argc > 1) {
		evbuffer_add_printf(client->buffer, "ERR - Too many arguments (expected 0 or 1)\n");
		return;
	}

	if(argc == 1) {
		int d = atoi(args);

		if(d < 0 || d >= 2048) {
			evbuffer_add_printf(client->buffer, "ERR - Raw distance value %d is out of range (0-2047).\n", d);
			return;
		}

		evbuffer_add_printf(client->buffer, "OK - %d -> %dmm -> %fmm^2.\n",
				d, depth_lut[d], surface_lut[d]);
	} else {
		evbuffer_add_printf(client->buffer, "OK - 2048 lines follow\n");
		for(i = 0; i < 2048; i++) {
			evbuffer_add_printf(client->buffer, "%e\n", surface_lut[i]);
		}
	}
}

#ifdef DEBUG
static void die_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	evbuffer_add_printf(client->buffer, "OK - Stopping server\n");
	client->server->info->stop = 1;
}

static void segv_func(struct knd_client *client, struct knd_cmd *command, int argc, const char *args)
{
	// Write directly to the client's fd since the server is about to crash
	const char * const msg = "OK - Crashing server\n";
	if(write(client->fd, msg, strlen(msg)) != (ssize_t)strlen(msg)) {
		ERRNO_KNDSRV(client, "Error writing crash message to client.\n");
	}
	*(volatile int *)NULL = 0;
}
#endif /* DEBUG */


/*
 * Parses an individual command line.  The contents of the line are modified
 * during parsing.  Responses are sent to the given socket.  Socket errors are
 * not reported (it is expected that they will be caught in the outer socket
 * loop).
 */
static void parse_line(struct knd_client *client, char *line)
{
	char *cmd, *args;
	size_t line_len;
	size_t cmd_len;
	size_t args_len;
	size_t args_count;
	unsigned int i;

	// TODO: Client API that speaks this protocol to a server

	line_len = strlen(line);

	cmd_len = strcspn(line, " ");

	line[cmd_len] = 0;
	cmd = line;

	// FIXME: Support escaping of argument separator in string values
	args = line + MIN_NUM(cmd_len + 1, line_len);
	args_len = strlen(args);
	args_count = nl_strcount(args, ',') + MIN_NUM(args_len, 1); // If args_len is 0, args_count will not have 1 added

	// TODO: Allow the user to assign a sequence number to each incoming
	// command, and use an internal sequence number, to allow asynchronous
	// command completion, track return values, and identify any dropped
	// commands?
	for(i = 0; i < ARRAY_SIZE(commands); i++) {
		if(!strcmp(commands[i].name, cmd)) {
			commands[i].func(client, &commands[i], args_count, args);
			break;
		}
	}
	if(i == ARRAY_SIZE(commands)) {
		DEBUG_KNDSRV(client, "Unknown command\n");
		evbuffer_add_printf(client->buffer, "ERR - Unknown command\n");
	}

	return;
}

/*
 * Parses command lines from the given client's input buffer, calling
 * parse_line() for each line found.  The client buffer is modified as
 * part of parsing.  If a newline is not encountered at the end of the buffer,
 * the remaining data is moved to the beginning of the buffer.  Returns the
 * amount of data left over after parsing all complete lines.
 */
static size_t parse_buffer(struct knd_client *client, size_t count) // TODO: Use evbuffers
{
	char *buf = client->in_buffer;
	char *tmp;
	char *last;
	size_t len;
	int i = 0;

	count += client->last_len;

	last = buf;
	do {
		if(i == 0) {
			// FIXME: Come up with a cleaner/better way of using last_len for the first iteration
			tmp = nl_strnpbrk(last + client->last_len, "\r\n", count - (last + client->last_len - buf));
		} else {
			tmp = nl_strnpbrk(last, "\r\n", count - (last - buf));
		}

		if(tmp == last) {
			// Found a newline character right away (perhaps \r\n)
			*tmp = 0;
			tmp++;
			last = tmp;
		} else if(tmp != NULL) {
			*tmp = 0;
			tmp++;
			parse_line(client, last);
			last = tmp;
		}

		i++;
	} while(tmp != NULL && !client->shutdown_requested);
	// TODO: Reintroduce MAX_LINES_PER_CYCLE and queue another event on the
	// loop to handle the remaining lines so that a single client can't
	// starve the event loop

	len = count - (last - buf);
	client->last_len = len;
	if(len) {
		if(last != buf) {
			memmove(buf, last, len);
		}
	}

	return len;
}

/*
 * Converts an inet6 address to its string equivalent (uses IPv4 form for
 * IPv4-in-IPv6 addresses).  The returned string must be free()d.  Returns NULL
 * on error.
 */
static char *addr_to_string(struct sockaddr_in6 *addr6)
{
	char addr[INET6_ADDRSTRLEN];
	char addr2[INET6_ADDRSTRLEN + 2];
	const char *addr_start;

	addr_start = inet_ntop(addr6->sin6_family, &addr6->sin6_addr, addr, sizeof(addr));
	if(!strncmp(addr, "::ffff:", 7) && strchr(addr, '.') != NULL) {
		return strdup(addr_start + 7);
	}

	snprintf(addr2, ARRAY_SIZE(addr2), "[%s]", addr);
	return strdup(addr2);
}

/*
 * Inserts a client structure into the list of clients.
 */
static void add_client(struct knd_server *server, struct knd_client *client)
{
	client->prev = server->client_list;
	client->next = server->client_list->next;
	if(client->next != NULL) {
		client->next->prev = client;
	}
	server->client_list->next = client;
	server->client_count++;
}

static struct knd_client *create_client(struct knd_server *server, int sockfd, char *remote_addr, unsigned short port)
{
	struct knd_client *client;

	client = calloc(1, sizeof(struct knd_client));
	if(client == NULL) {
		ERRNO_OUT("Error allocating command handler info for client %s:%hu on fd %d",
				remote_addr, port, sockfd);
		close(sockfd);
		return NULL;
	}
	client->fd = sockfd;
	client->remote_addr = remote_addr;
	client->remote_port = ntohs(port);
	client->server = server;

	add_client(server, client);

	return client;
}

static void free_client(struct knd_client *client)
{
	struct knd_server *server;

	if(CHECK_NULL(client)) {
		abort();
	}

	server = client->server;

	// Remove socket info from list of sockets
	if(client->prev->next == client) {
		client->prev->next = client->next;
	} else {
		ERROR_OUT("BUG: Socket list is inconsistent: client->prev->next != client!\n");
		abort();
	}
	if(client->next != NULL) {
		if(client->next->prev == client) {
			client->next->prev = client->prev;
		} else {
			ERROR_OUT("BUG: Socket list is inconsistent: client->next->prev != client!\n");
			abort();
		}
	}

	// Close socket and free resources
	if(client->buf_event != NULL) {
		bufferevent_free(client->buf_event);
	}
	if(client->buffer != NULL) {
		evbuffer_free(client->buffer);
	}
	if(client->remote_addr != NULL) {
		free(client->remote_addr);
	}
	if(client->fd >= 0) {
		shutdown_client(client);
		if(close(client->fd)) {
			ERRNO_OUT("Error closing connection on fd %d", client->fd);
		}
	}
	if(client->zones != NULL) {
		destroy_zonelist(client->zones);
	}
	free(client);

	server->client_count--;
}

/*
 * Sets all bits in fl in the given file descriptor's flags using F_SETFL.
 */
static int set_flags(int fd, int fl)
{
	int flags;

	flags = fcntl(fd, F_GETFL);
	if(flags == -1) {
		ERRNO_OUT("Error getting flags on fd %d", fd);
		return -1;
	}
	flags |= fl;
	if(fcntl(fd, F_SETFL, flags)) {
		ERRNO_OUT("Error setting non-blocking I/O on fd %d", fd);
		return -1;
	}

	return 0;
}

static void knd_read(struct bufferevent *buf_event, void *arg)
{
	struct knd_client *client = (struct knd_client *)arg;
	size_t len;

	// Ignore data if the client connection is, or should be, shut down
	if(client->shutdown_requested) {
		return;
	}

	len = bufferevent_read(buf_event, client->in_buffer + client->last_len, ARRAY_SIZE(client->in_buffer) - client->last_len);
	if(parse_buffer(client, len) == ARRAY_SIZE(client->in_buffer)) {
		ERROR_OUT("Client buffer is full on fd %d with length %zu.  Closing connection.\n", client->fd, client->last_len);
		client->in_buffer[ARRAY_SIZE(client->in_buffer) - 1] = 0;
		evbuffer_add_printf(client->buffer, "\n\n\nBuffer overflow.\n\n\n");
		request_shutdown_client(client);
		flush_client(client);
	}

	// Send the results to the client
	flush_client(client);
}

static void knd_write(struct bufferevent *buf_event, void *arg)
{
	struct knd_client *client = (struct knd_client *)arg;

	if(client->shutdown_requested && EVBUFFER_LENGTH(buf_event->output) == 0) {
		shutdown_client(client);
	}
}

static void knd_error(struct bufferevent *buf_event, short error, void *arg)
{
	struct knd_client *client = (struct knd_client *)arg;

	if(error & EVBUFFER_EOF) {
		/*I*/ERROR_OUT("Client %s disconnected from fd %d.\n", client->remote_addr, client->fd);
		client->shutdown = 1;
	} else if(error & EVBUFFER_TIMEOUT) {
		/*I*/ERROR_OUT("Client %s:%hu on fd %d timed out.\n",
				client->remote_addr, client->remote_port, client->fd);
		evbuffer_add_printf(client->buffer, "Timeout.\n");
		request_shutdown_client(client);
		flush_client(client);
		// TODO: Distinguish between read and write timeouts (if timeouts are ever used)
	} else if(error & EVBUFFER_WRITE) {
		ERROR_OUT("Error writing to %s on fd %d.\n", client->remote_addr, client->fd);
		client->shutdown = 1;
	} else {
		ERROR_OUT("A socket error (0x%hx) occurred on fd %d.\n", error, client->fd);
		client->shutdown = 1;
	}

	if(client->shutdown) {
		free_client(client);
	}
}

// Compatibility shim for libevent 1.4 through libevent 2.x
// See https://github.com/libevent/libevent/pull/678
// See also https://github.com/nitrogenlogic/nlutils/blob/5612cd1592277913b101b93dc5183b13e40edcc8/src/url_req.c#L965-L983
static struct bufferevent *create_bufferevent(struct knd_server *server, struct knd_client *client, int fd)
{
	struct bufferevent *newbuf;
#if defined(EVENT__NUMERIC_VERSION) && EVENT__NUMERIC_VERSION >= 0x02000000
	// libevent 2 (libevent 2.1 introduced a segfault in bufferevent_new())
	newbuf = bufferevent_socket_new(server->evloop, fd, 0);
	if(newbuf != NULL) {
		bufferevent_setcb(newbuf, knd_read, knd_write, knd_error, client);
	}
#else
	// libevent 1.4
	newbuf = bufferevent_new(fd, knd_read, knd_write, knd_error, client);
#endif

	return newbuf;
}

static void setup_connection(int sockfd, struct sockaddr_in6 *remote_addr, struct knd_server *server)
{
	struct knd_client *client;
	char *addr = addr_to_string(remote_addr);

	if(addr == NULL) {
		ERROR_OUT("Error converting client address to string for connection on fd %d\n", sockfd);
		return;
	}

	/*I*/ERROR_OUT("Client %s connected on fd %d\n", addr, sockfd);

	if(set_flags(sockfd, O_NONBLOCK)) {
		ERROR_OUT("Error setting non-blocking I/O on an incoming connection.\n");
		free(addr);
		close(sockfd);
	}

	// Copy connection info into a command handler info structure
	client = create_client(server, sockfd, addr, remote_addr->sin6_port);
	if(client == NULL) {
		free(addr);
		close(sockfd);
		return;
	}

	// Initialize a buffered I/O event
	client->buf_event = create_bufferevent(server, client, sockfd);
	if(CHECK_NULL(client->buf_event)) {
		ERROR_OUT("Error initializing buffered I/O event for fd %d.\n", sockfd);
		free_client(client);
		return;
	}
	bufferevent_base_set(server->evloop, client->buf_event);
	bufferevent_settimeout(client->buf_event, CLIENT_TIMEOUT, 0);
	if(bufferevent_enable(client->buf_event, EV_READ)) {
		ERROR_OUT("Error enabling buffered I/O event for fd %d.\n", sockfd);
		free_client(client);
		return;
	}

	// Create the outgoing data buffer
	client->buffer = evbuffer_new();
	if(CHECK_NULL(client->buffer)) {
		ERROR_OUT("Error creating output buffer for fd %d.\n", sockfd);
		free_client(client);
		return;
	}
}

static void knd_connect(int listenfd, short evtype, void *arg)
{
	struct sockaddr_in6 remote_addr;
	socklen_t addrlen = sizeof(remote_addr);
	int sockfd;
	int i;

	if(!(evtype & EV_READ)) {
		ERROR_OUT("Unknown event type in connect callback: 0x%hx\n", evtype);
		return;
	}

	// Accept and configure incoming connections (up to QUEUED_CONNECTIONS connections in one go)
	for(i = 0; i < QUEUED_CONNECTIONS; i++) {
		sockfd = accept(listenfd, (struct sockaddr *)&remote_addr, &addrlen);
		if(sockfd < 0) {
			if(errno != EWOULDBLOCK && errno != EAGAIN) {
				ERRNO_OUT("Error accepting an incoming connection");
			}
			break;
		}

		setup_connection(sockfd, &remote_addr, (struct knd_server *)arg);
	}
}

// Used by process_subscriptions to iterate over the list of zones
static void subs_callback(void *data, struct zone *zone)
{
	struct knd_client *client = data;

	// It is extremely unlikely that any parameter (such as center of
	// gravity) will change without pop also changing, due to the high
	// noise levels present at the fringes of objects.  However, we
	// also need to check occupied because it can change from 1 to 0
	// long after pop stops changing, due to rising/falling delay logic.
	if(zone->lastpop != zone->pop || zone->lastoccupied != zone->occupied || zone->new_zone) {
		evbuffer_add(client->buffer, "SUB - ", 6);
		send_zone_info(client, zone, zone->new_zone);
	}
}

static void depthsub_callback(uint8_t *buf, void *data)
{
	struct knd_client *client = data;
	evbuffer_add(client->buffer, buf, FREENECT_DEPTH_11BIT_PACKED_SIZE);
}

/*
 * Called by the wakeup pipe handler for each client after kndsrv_send_depth() has
 * been called.
 */
static void process_subscriptions(struct knd_client *client)
{
	if(client->subglobal) {
		// TODO: Generate the text once and send it to all subscribed clients.
		iterate_zonelist(client->server->info->zones, subs_callback, client);
	}
	if(client->subdepth) {
		if(client->depth_limit > 0) {
			if(--client->depth_limit == 0) {
				client->subdepth = 0;
			}
		}

		evbuffer_add_printf(client->buffer, "DEPTH - %d bytes of raw data follow newline\n",
				FREENECT_DEPTH_11BIT_PACKED_SIZE);
		if(get_depth(client->server->info->vid, depthsub_callback, client)) {
			ERROR_KNDSRV(client, "Error getting depth data.\n");
			request_shutdown_client(client);
		}
	}

	flush_client(client);
}

static void videosub_callback(uint8_t *buf, void *data)
{
	struct knd_client *client = data;
	evbuffer_add(client->buffer, buf, KND_VIDEO_SIZE);
}

// Used by process_video to iterate over the list of zones
static void bright_callback(void *data, struct zone *zone)
{
	struct knd_client *client = data;

	// TODO: escape name
	evbuffer_add_printf(client->buffer, "BRIGHT - bright=%d name=\"%s\"\n",
			zone->bsum * 256 / zone->maxpop, zone->name);
}

/*
 * Called by the wakeup pipe handler for each client after kndsrv_send_video()
 * has been called.
 */
static void process_video(struct knd_client *client)
{
	if(client->subbright) {
		// TODO: Generate the text once and send it to all subscribed clients.
		iterate_zonelist(client->server->info->zones, bright_callback, client);
		client->subbright = 0;
	}
	if(client->subvideo) {
		evbuffer_add_printf(client->buffer, "VIDEO - %d bytes of video data follow newline\n",
				KND_VIDEO_SIZE);

		if(get_video(client->server->info->vid, videosub_callback, client)) {
			ERROR_KNDSRV(client, "Error getting video data.\n");
			request_shutdown_client(client);
		}

		client->subvideo = 0;
	}

	flush_client(client);
}

/*
 * Writes a depth wakeup instruction to the given server's wakeup pipe.  Call
 * this when a depth frame is received.
 */
void kndsrv_send_depth(struct knd_server *server)
{
	errno = 0;
	if(write(server->wake_write, "Z", 1) != 1) {
		ERRNO_OUT("Error writing depth wakeup to server wakeup pipe");
	}
}

/*
 * Writes a video wakeup instruction to the given server's wakeup pipe.  Call
 * this when a video frame is received.
 */
void kndsrv_send_video(struct knd_server *server)
{
	errno = 0;
	if(write(server->wake_write, "V", 1) != 1) {
		ERRNO_OUT("Error writing video wakeup to server wakeup pipe");
	}
}

/*
 * Writes a kill instruction to the given server's wakeup pipe.
 */
static void kndsrv_send_kill(struct knd_server *server)
{
	errno = 0;
	if(write(server->wake_write, "K", 1) != 1) {
		ERRNO_OUT("Error writing kill command to server wakeup pipe");
	}
}

/*
 * Handles notification from the image processing thread (via
 * kndsrv_send_depth()) that it's time to update subscriptions or shut down.
 */
static void knd_wake(int fd, short evtype, void *arg)
{
	struct knd_server *server = arg;
	struct knd_client *client;
	char buf[1024];
	ssize_t depthcount = 0;
	ssize_t videocount = 0;
	ssize_t ret;
	ssize_t i;

	do {
		ret = read(server->wake_read, buf, ARRAY_SIZE(buf));
		for(i = 0; i < ret; i++) {
			if(buf[i] == 'K') {
				event_base_loopexit(server->evloop, NULL);
				depthcount = 0;
				break;
			} else if(buf[i] == 'V') {
				videocount++;
			} else {
				depthcount++;
			}
		}
	} while(ret > 0);

	if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
		ERRNO_OUT("Error reading from event loop wakeup pipe");
	}

	if(depthcount) {
		client = server->client_list->next;
		while(client != NULL) {
			process_subscriptions(client);
			client = client->next;
		}
		touch_zonelist(server->info->zones);
	}

	if(videocount) {
		client = server->client_list->next;
		while(client != NULL) {
			process_video(client);
			client = client->next;
		}
	}
}

/*
 * Creates a server for the given knd context.  Call kndsrv_run() to run the
 * server's event loop.  kndsrv_run() should be called shortly after the server
 * is created, as the socket begins waiting for connections immediately.  Call
 * kndsrv_stop() in another thread, an event handler, or a signal handler to
 * stop the running server.  Pass 0 for port to use the default port.  Returns
 * NULL on error.
 */
struct knd_server *kndsrv_create(struct knd_info *info, unsigned short port)
{
	struct knd_server *server = NULL;
	struct sockaddr_in6 local_addr;
	int wakefds[2] = {-1, -1};
	int ret;

	if(info == NULL) {
		ERROR_OUT("Cannot create a server for a null knd info.\n");
		goto error;
	}

	if(port == 0) {
		port = KND_PORT;
	}

	server = calloc(1, sizeof(struct knd_server));
	if(server == NULL) {
		ERRNO_OUT("Error allocating memory for server");
		goto error;
	}

	server->client_list = calloc(1, sizeof(struct knd_client));
	if(server->client_list == NULL) {
		ERRNO_OUT("Error allocating memory for server's client list");
		goto error;
	}

	server->info = info;

	// Create wakeup pipe
	ret = pipe(wakefds);
	server->wake_read = wakefds[0];
	server->wake_write = wakefds[1];
	if(ret) {
		ERRNO_OUT("Error creating event loop wakeup pipe.");
		goto error;
	}

	if(set_flags(server->wake_read, O_NONBLOCK)) {
		ERROR_OUT("Error setting event loop wakeup read pipe for non-blocking I/O.\n");
		goto error;
	}

	// Initialize libevent
	server->evloop = event_base_new();
	if(server->evloop == NULL) {
		ERROR_OUT("Error initializing event loop.\n");
		goto error;
	}

	// Initialize socket address
	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin6_family = AF_INET6;
	local_addr.sin6_port = htons(port);
	local_addr.sin6_addr = in6addr_any;

	// Begin listening for connections (TODO: move this to kndsrv_run()?)
	server->listenfd = socket(AF_INET6, SOCK_STREAM, 0);
	if(server->listenfd == -1) {
		ERRNO_OUT("Error creating listening socket");
		goto error;
	}
	int tmp_reuse = 1;
	if(setsockopt(server->listenfd, SOL_SOCKET, SO_REUSEADDR, &tmp_reuse, sizeof(tmp_reuse))) {
		ERRNO_OUT("Error enabling socket address reuse on listening socket");
		goto error;
	}
	if(bind(server->listenfd, (struct sockaddr *)&local_addr, sizeof(local_addr))) {
		ERRNO_OUT("Error binding listening socket");
		goto error;
	}
	if(listen(server->listenfd, QUEUED_CONNECTIONS)) {
		ERRNO_OUT("Error listening to listening socket");
		goto error;
	}

	// Set socket for non-blocking I/O so multiple connections can be
	// accepted per connect callback
	if(set_flags(server->listenfd, O_NONBLOCK)) {
		ERROR_OUT("Error setting listening socket to non-blocking I/O.\n");
		goto error;
	}

	// Add an event to wait for connections (allocated separately to allow
	// inclusion of knd_server.h without libevent headers)
	server->connect_event = calloc(1, sizeof(struct event));
	if(server->connect_event == NULL) {
		ERRNO_OUT("Error allocating memory for server connection event");
		goto error;
	}

	event_set(server->connect_event, server->listenfd, EV_READ | EV_PERSIST, knd_connect, server);
	event_base_set(server->evloop, server->connect_event);
	if(event_add(server->connect_event, NULL)) {
		ERROR_OUT("Error scheduling connection event on the event loop.\n");
		goto error;
	}

	// Add event to watch for wakeup requests from info callback
	server->wake_event = calloc(1, sizeof(struct event));
	if(server->wake_event == NULL) {
		ERRNO_OUT("Error allocating memory for server wakeup event");
		goto error;
	}

	event_set(server->wake_event, server->wake_read, EV_READ | EV_PERSIST, knd_wake, server);
	event_base_set(server->evloop, server->wake_event);
	if(event_add(server->wake_event, NULL)) {
		ERROR_OUT("Error scheduling wakeup event on the event loop.\n");
		goto error;
	}

	return server;

error:
	if(server != NULL) {
		kndsrv_destroy(server);
	}

	return NULL;
}

/*
 * Shuts down and frees all client connections on the given server.
 */
static void knd_free_clients(struct knd_server *server)
{
	while(server->client_list->next != NULL) {
		free_client(server->client_list->next);
	}
}

/*
 * Destroys the given server.  This should not be called while the server's
 * event loop is running.  Instead, call kndsrv_stop(), then call
 * kndsrv_destroy() when kndsrv_stop() returns.
 */
void kndsrv_destroy(struct knd_server *server)
{
	if(CHECK_NULL(server)) {
		ERROR_OUT("Cannot destroy a null server.\n");
		return;
	}

	if(server->connect_event != NULL) {
		if(event_del(server->connect_event)) {
			ERROR_OUT("Error removing connection event from the event loop.\n");
		}
		free(server->connect_event);
	}
	if(server->wake_event != NULL) {
		if(event_del(server->wake_event)) {
			ERROR_OUT("Error removing wakeup event from the event loop.\n");
		}
		free(server->wake_event);
	}

	if(server->evloop != NULL) {
		event_base_free(server->evloop);
	}
	if(server->listenfd >= 0) {
		if(close(server->listenfd)) {
			ERRNO_OUT("Error closing listening socket");
		}
	}
	if(server->client_list != NULL) {
		knd_free_clients(server);
		free(server->client_list);
	}
	if(server->wake_read >= 0) {
		if(close(server->wake_read)) {
			ERRNO_OUT("Error closing server's wakeup read fd.\n");
		}
	}
	if(server->wake_write >= 0) {
		if(close(server->wake_write)) {
			ERROR_OUT("Error closing server's wakeup write fd.\n");
		}
	}

	free(server);
}

/*
 * The server event loop thread.
 */
static void *kndsrv_thread(void *d)
{
	struct knd_server *server = d;

	nl_set_threadname("kndsrv_thread");

	// Start the event loop
	if(event_base_dispatch(server->evloop)) {
		ERROR_OUT("Error running event loop.\n");
	}

	// Clean up and close open connections
	knd_free_clients(server);

	return NULL;
}

/*
 * Starts the given server's event loop in a newly-created thread.  Returns 0
 * on success, -1 on error.
 */
int kndsrv_run(struct knd_server *server)
{
	int ret;

	if(CHECK_NULL(server)) {
		ERROR_OUT("Cannot run a null server.\n");
		return -1;
	}

	ret = nl_create_thread(server->info->thread_ctx, NULL, kndsrv_thread, server, "kndsrv_thread", &server->thread);
	if(ret) {
		ERROR_OUT("Error starting server thread: %s\n", strerror(ret));
		return -1;
	}

	return 0;
}

/*
 * Stops the given server's event loop.  This function waits for the server
 * event thread to exit.
 */
void kndsrv_stop(struct knd_server *server)
{
	int ret;

	kndsrv_send_kill(server);

	ret = nl_join_thread(server->thread, NULL);
	if(ret) {
		ERROR_OUT("Error joining server thread: %s\n", strerror(ret));
	}
}

