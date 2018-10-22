file(COPY knd_monitor.sh
	DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

install(PROGRAMS knd_monitor.sh
	DESTINATION "${INSTALLDIR}/opt/nitrogenlogic/knd/")

install(FILES knd.conf
	DESTINATION "${INSTALLDIR}/etc/nitrogenlogic/knd/")

install(DIRECTORY DESTINATION "${INSTALLDIR}/var/lib/knd/")
