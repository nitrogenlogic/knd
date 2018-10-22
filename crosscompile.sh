#!/bin/bash
# Cross-compilation script for knd.
# Copyright (C)2011-2018 Mike Bourgeous.  Released under AGPLv3 in 2018.

CROSS_BASE=${HOME}/devel/crosscompile
NCPUS=$(grep -i 'processor.*:' /proc/cpuinfo | wc -l)

case "$1" in
	neon)
	TOOLCHAIN=${HOME}/devel/nlutils/meta/toolchain/cmake-toolchain-arm-linux-neon.cmake
	LIBS_ROOT=${CROSS_BASE}/cross-libs-arm-neon
	ROOT=${CROSS_BASE}/cross-root-arm-neon-depth
	DEBIAN_ROOT=${CROSS_BASE}/debian-squeeze-root-armel-build
	;;

	nofp)
	TOOLCHAIN=${HOME}/devel/nlutils/meta/toolchain/cmake-toolchain-arm-linux-nofp.cmake
	LIBS_ROOT=${CROSS_BASE}/cross-libs-arm-nofp
	ROOT=${CROSS_BASE}/cross-root-arm-nofp-depth
	DEBIAN_ROOT=${CROSS_BASE}/debian-squeeze-root-armel-build
	;;

	*)
	echo "Not gonna work.  You gotta say \"neon\" or \"nofp\"."
	exit
	;;
esac

LIBS_PREFIX=${ROOT}/usr/local
PREFIX=${ROOT}/usr/local

# These variables are needed by the CMake toolchain files
export DEBIAN_ROOT
export LIBS_ROOT
export ROOT

# Install to embedded root
rm -rf build-$1
mkdir build-$1
cd build-$1
cmake \
	-D CMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} \
	-D CMAKE_INSTALL_PREFIX=${PREFIX} \
	-D CMAKE_BUILD_TYPE=Release \
	-D INSTALLDIR=${ROOT} \
	-D LIBUSB_1_INCLUDE_DIR=${DEBIAN_ROOT}/usr/include/libusb-1.0 \
	-D LIBUSB_1_LIBRARY=${DEBIAN_ROOT}/lib/libusb-1.0.so.0 \
	..

make -j$NCPUS
make -j$NCPUS install
cd ..
