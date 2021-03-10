#!/bin/bash
# Script to generate a Debian package for knd.
# Copyright (C)2018 Mike Bourgeous.  Released under AGPLv3 in 2018.

# Configuration vars
NAME="knd"
PKGNAME="knd"
DESCRIPTION="Kinematic Network Daemon (core depth sensor service for Nitrogen Logic controllers)"
PKGDEPS="libnlutils, libfreenect0.5, libevent-2.1-6, libusb-1.0-0"

# Build vars
BASEDIR="$(readlink -m "$(dirname "$0")/..")"
VER=$(grep 'set(KND_VERSION' "${BASEDIR}/CMakeLists.txt" | egrep -o '[0-9]+\.[0-9]+\.[0-9]+')
REL=$(($(cat "$BASEDIR/meta/_RELEASE") + 1))
VERSION="$VER-$REL"


if [ -r /usr/local/share/nlutils/pkg_helper.sh ]; then
	. /usr/local/share/nlutils/pkg_helper.sh
elif [ -r /usr/share/nlutils/pkg_helper.sh ]; then
	. /usr/share/nlutils/pkg_helper.sh
else
	printf "\033[1;31mCan't find nlutils package helper script\033[0m\n"
	exit 1
fi


# Save bumped release number
printf "\nBuild complete; saving release number\n"
echo -n $REL > "$BASEDIR/meta/_RELEASE"
git commit -m "Build package $VERSION" "$BASEDIR/meta/_RELEASE"
