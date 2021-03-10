# knd

The Kinematic Network Daemon from Nitrogen Logic's [Depth Camera
Controller][3].  This application runs in the background to provide a
convenient interface for a Kinect camera or similar depth sensor supported by
libfreenect.  KND has all the core graphics processing code, written in C,
while the KNC project provides the web UI, written in Ruby.

The sensor's visible area can be divided into 3D zones with specified
dimensions, with `knd` reporting information about objects within each zone.
`knd` operates at the voxel level, with no blob tracking or skeletal tracking,
allowing fast operation on low-powered devices.

The Depth Camera Controller web interface, the [Automation Controller][4] logic
system, the [PC Remote][5], and any custom integrations use a TCP/IP socket to
interact with `knd`.  See the API section below.

`knd` uses all integer math (with a few carefully chosen approximations and
optimizations, such as reciprocal multiplication) to support embedded devices
without floating point processors.


# Copying

Copyright 2011-2018 Mike Bourgeous, licensed under [GNU Affero GPLv3][6].  See
LICENSE for the complete text of the license.

Why AGPLv3?  Because this code is being released for historical preservation
and probably shouldn't be used for new development.  If you want more
permissive terms, just get in touch.


# Building

## Dependencies

You'll need to install a few Debian/Ubuntu packages:

```bash
sudo apt-get install libfreenect-dev libusb-1.0-0-dev libevent-dev
```

You'll also need to build and install [nlutils][0].

## Compiling

Use the provided Makefile to start a parallel compilation with CMake in an
architecture-specific subdirectory:

```bash
make
sudo make install # optional
```

You can build a Debian package with `meta/make_pkg.sh`, which uses package
helper scripts from nlutils.  See [the packaging section of the nlutils
README][2] for more info.

```bash
PKGDIR=/var/tmp meta/make_pkg.sh # default output directory is /tmp
```

## Cross-compilation

You'll need to cross-compile [nlutils][0] and [Nitrogen Logic's fork of libfreenect][1] first.

```bash
# Build for BeagleBoard
./crosscompile.sh neon

# Build for SheevaPlug
./crosscompile.sh nofp
```

You can also cross-build Debian packages, using the aforementioned packaging
tools from nlutils:

```bash
# In nlutils
meta/make_root.sh

# In knd
meta/cross_pkg.sh
```


# Running

To run `knd` on your local machine, you need to unload the Video4Linux driver
for the Kinect and specify the data directory for `knd` to use:

```bash
sudo rmmod gspca_kinect
mkdir $HOME/.knd/
KND_SAVEDIR=$HOME/.knd/ ./build-$(uname -m)/src/knd
```

If there is no Kinect camera present, if the current user doesn't have
permission to access it, or if the camera takes too long to initialize, `knd`
will not start and an error will be displayed.

On an original Nitrogen Logic Depth Camera Controller, `knd` is run by
`knd_monitor.sh`, started by a line in `/etc/inittab`.  This line is added by
the post-firmware-update script in the `knc` project.  `knd_monitor.sh` makes
sure the V4L driver for the Kinect is not loaded, then runs `knd` in a loop.
`knd` is designed to exit when anything goes wrong, so this loop recovers from
errors (such as a Kinect being unplugged and plugged back in) and keeps `knd`
running.


# API

If a Kinect camera is present, `knd` provides a machine-friendly TCP/IP
command-line interface on port 14308.  You can connect with Netcat and get a
list of supported commands by typing `help` (replace `localhost` with the name
of your controller if you are not running locally):

```bash
nc localhost 14308
```

```
help
```

There are some examples of parsing the API in shell scripts in `examples/`.


## Commands with example responses

Parameters to commands are comma-separated *without whitespace*.  Parameters
may not contain a comma (so zone names cannot contain commas or newlines).

API responses always start with one line of text.  Each line will start with
`OK` for an acknowledgement of a command, `ERR` if a command was invalid, `SUB`
for a periodic response to the `sub` command, `BRIGHT` for a response to the
`getbright` command, `DEPTH` for a depth frame, `VIDEO` for a video frame.
Multi-line or binary responses will contain the number of lines or number of
bytes to read immediately following the response line.

Commands will be run in the order they are received, but some commands trigger
the generation of delayed responses.  Clients will need to handle any line
received at any time, based on its prefix, but `OK` and `ERR` lines will always
be returned in the order commands were sent.

Multi-value responses (e.g. SUB lines) are a sequence of key-value pairs.
String values may optionally be quoted.  Lists of key-value pairs may be parsed
using the `nl_parse_kvp()` function from [libnlutils][0].  Quoted strings may
be processed using the `nl_unescape_string()` function from [libnlutils][0].

Dimensions for the `addzone` and `subzone` commands are in millimeters relative
to the depth sensor itself.  When looking at the front of the depth sensor,
positive X points to the right, positive Y points upward toward the ceiling,
and positive Z points away from the sensor toward the room.

- **bye**
  ```
  OK - Goodbye
  ```
- **ver**
  ```
  OK - Version 2
  ```
- **help**
  ```
  OK - 19 commands (app version 0.1.0)
  bye - Disconnects from the server.
  ver - Returns the server protocol version.
  help - Lists available commands.
  addzone - Adds a new global zone (name, xmin, ymin, zmin, xmax, ymax, zmax).
  setzone - Sets a zone's parameters (name, all, xmin, ymin, zmin, xmax, ymax, zmax or name, [attr], value).
  rmzone - Removes a global zone (name).
  clear - Removes all global zones.
  zones - Lists all global zones.
  sub - Subscribe to global zone updates.
  unsub - Unsubscribe from global zone updates.
  getdepth - Grabs a single 11-bit packed depth image (increments subscription count if already subscribed).
  subdepth - Subscribes to 11-bit packed depth data (count (optional, <=0 means forever)).
  unsubdepth - Unsubscribes from 11-bit packed depth data.
  getbright - Asynchronously returns the approximate brightness within each zone.
  getvideo - Grabs a single video image.
  tilt - Sets or returns the camera tilt in degrees from horizontal.
  fps - Returns the approximate frame rate (updated every 200ms).
  lut - Returns the depth look-up table, or looks up an entry in the table.
  sa - Returns the surface area look-up table, or looks up an entry in the table.
  ```
- **addzone Living,1,1,1,2,2,2**
  ```
  OK - Zone "Living" was added.
  ```
- **setzone Zone 6,px_xmin,1**
  ```
  OK - Zone "Zone 6" attribute "px_xmin" was updated.
  ```
- **rmzone Zone 6**
  ```
  OK - Zone "Zone 6" was removed.
  ```
- **clear**
  ```
  OK - All zones were removed.
  ```
- **zones** (the version is the zone list version, not the application version)
  ```
  OK - 1 zones - Version 9, 0 occupied, peak zone is -1 "[none]"
  xmin=1 ymin=1 zmin=1 xmax=2 ymax=2 zmax=2 px_xmin=0 px_ymin=0 px_zmin=0 px_xmax=20 px_ymax=0 px_zmax=0 negate=0 param=pop on_level=160 off_level=140 on_delay=1 off_delay=1 occupied=0 pop=0 maxpop=1 xc=-1 yc=-1 zc=-1 sa=0 name="Zone2"
  ```
- **sub** (SUB lines will be received at the start and every time a zone changes)
  ```
  OK - Subscribed to global zone updates
  SUB - xmin=1 ymin=1 zmin=1 xmax=2 ymax=2 zmax=2 px_xmin=0 px_ymin=0 px_zmin=0 px_xmax=20 px_ymax=0 px_zmax=0 negate=0 param=pop on_level=160 off_level=140 on_delay=1 off_delay=1 occupied=0 pop=0 maxpop=1 xc=-1 yc=-1 zc=-1 sa=0 name="Zone2"
  ```
- **unsub**
  ```
  OK - Unsubscribed from global zone updates
  ```
- **getdepth** (other lines may be received before the DEPTH line)
  ```
  OK - Requested a single depth frame for delivery as a DEPTH message
  DEPTH - 422400 bytes of raw data follow newline
  [raw data]
  ```
- **subdepth**
  ```
  OK - depth frames will be delivered as DEPTH messages until unsubscribed
  DEPTH - 422400 bytes of raw data follow newline
  [raw data]
  ```
- **unsubdepth**
  ```
  OK - Unsubscribed from depth data
  ```
- **getbright** (other lines may be received before the BRIGHT line)
  ```
  OK - Requested brightness for each zone
  BRIGHT - bright=1536 name="Zone2"
  ```
- **getvideo** (other lines may be received before the VIDEO line)
  ```
  OK - Requested delivery of a video frame
  VIDEO - 307200 bytes of video data follow newline
  [raw data]
  ```
- **tilt**
  ```
  OK - Current tilt is 0 degrees
  ```
- **tilt 5**
  ```
  OK - Requested tilt of 5 degrees
  ```
- **fps**
  ```
  OK - 28 fps
  ```


[0]: https://github.com/nitrogenlogic/nlutils
[1]: https://github.com/nitrogenlogic/libfreenect
[2]: https://github.com/nitrogenlogic/nlutils#debianubuntu-packages
[3]: http://www.nitrogenlogic.com/products/depth_controller.html
[4]: http://www.nitrogenlogic.com/products/automation_controller.html
[5]: http://www.nitrogenlogic.com/products/pc_remote.html
[6]: https://www.gnu.org/licenses/agpl-3.0.html
