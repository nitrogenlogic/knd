.PHONY: all build debug clean install init init-debug distclean

ncpus := $(shell grep -i 'processor.*:' /proc/cpuinfo | wc -l | sed -e 's/^0/8/')

all: build

build: init
	sh -c "cd build-$$(uname -m) && make -j$(ncpus)"

debug: init-debug
	sh -c "cd build-$$(uname -m) && make -j$(ncpus)"

clean: init
	sh -c "cd build-$$(uname -m) && make -j$(ncpus) clean"

install: init
	sh -c "cd build-$$(uname -m) && make -j$(ncpus) install && ldconfig"

init:
	sh -c "mkdir -p build-$$(uname -m)"
	sh -c "cd build-$$(uname -m) && cmake -DCMAKE_BUILD_TYPE=Release $(CMAKE_DEFS) .."

init-debug:
	sh -c "mkdir -p build-$$(uname -m)"
	sh -c "cd build-$$(uname -m) && cmake -DCMAKE_BUILD_TYPE=Debug $(CMAKE_DEFS) .."

distclean:
	sh -c "rm -rf build-$$(uname -m) build-pkg-*"
