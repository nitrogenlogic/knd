find_library(LIBEVENT_CORE_LIBRARY event_core HINTS /usr/local/lib)
find_library(LIBUSB_1_LIBRARY usb-1.0)
find_library(LIBNLUTILS_LIBRARY nlutils HINTS /usr/local/lib)

add_executable(apxtan apxtan.c)
target_link_libraries(apxtan m)

add_executable(knd knd.c inline_defs.c kndsrv.c save.c vidproc.c watchdog.c zone.c)
target_link_libraries(knd m freenect ${LIBNLUTILS_LIBRARY} ${LIBEVENT_CORE_LIBRARY} ${LIBUSB_1_LIBRARY})

install(TARGETS knd RUNTIME DESTINATION bin)
