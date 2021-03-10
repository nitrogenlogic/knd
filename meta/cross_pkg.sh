#!/bin/sh
# Script to build packages in a Debian cross-compilation root (e.g. for i386).
# Copyright (C)2015 Mike Bourgeous.  Relased under AGPLv3 in 2018.

# Debian architecture name
ARCH=${ARCH:-armel}

# Debian release name
RELEASE=buster

# Project directory
BASEDIR="$(readlink -m "$(dirname "$0")/..")"

EXTRA_PACKAGES="\
libfreenect-dev,
libevent-dev,
libusb-1.0-0-dev
"


if [ -r /usr/local/share/nlutils/build_root_helper.sh ]; then
	. /usr/local/share/nlutils/build_root_helper.sh
elif [ -r /usr/share/nlutils/build_root_helper.sh ]; then
	. /usr/share/nlutils/build_root_helper.sh
else
	printf "\033[1;31mCan't find nlutils build root helper script\033[0m\n"
	exit 1
fi
