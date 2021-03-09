set(KND_VERSION 1.0.0)

cmake_minimum_required(VERSION 2.6)

# Prevent project() from clearing VERSION variables
if(POLICY CMP0048)
	cmake_policy(SET CMP0048 OLD)
endif()

project(KND)

# Prevent absolute paths from being compiled in
set(CMAKE_USE_RELATIVE_PATHS true)

find_path(LIBFREENECT_INCLUDE_DIR libfreenect.h HINTS /usr/include/libfreenect /usr/local/include/libfreenect)
message("libfreenect include dir is ${LIBFREENECT_INCLUDE_DIR}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${LIBFREENECT_INCLUDE_DIR} --std=gnu99 -D_XOPEN_SOURCE=700 -D_GNU_SOURCE -fPIC -pthread -Wall -Wextra -Werror -Wno-cast-align -Wno-unused-parameter -pipe -DKND_VERSION='\"${KND_VERSION}\"'")

add_subdirectory(src)

if(NL_EMBEDDED)
	message("Embedded build; including embedded/ directory")
	add_subdirectory(embedded)
endif(NL_EMBEDDED)
