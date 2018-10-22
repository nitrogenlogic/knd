#!/bin/bash
# Copyright (C)2011 Mike Bourgeous.  Released under AGPLv3 in 2018.
# A script that uses the center of gravity feature to adjust light brightness
# based on hand position within a zone next to a lamp.  The zone details:
# 
# xmin=0.254 ymin=-0.352 zmin=3.094 xmax=0.468 ymax=0.208 zmax=3.444
# px_xmin=251 px_ymin=210 px_zmin=979 px_xmax=287 px_ymax=291 px_zmax=990
# occupied=0 pop=0 maxpop=2916 xc=0.000 yc=0.000 zc=0.000 name="Brightness"
# 
# The yc value is offset and scaled to the light value range (0-100), then sent
# to the automation controller.

echo Press Enter to stop this script.

(echo sub; read) | \
	nc localhost 14308 | \
	grep --line-buffered 'occupied=1.*name="Brightness"' | \
	grep --line-buffered -o 'yc=[^ ]*' | \
	sed -ue 's#yc=\(.*\)#(\1 + 0.018) * 100.0 / 0.53#' | \
	bc | tee /dev/stderr | \
	sed -ue 's/^\([^.]*\).*/set 13,4,\1/' | \
	nc logic-controller.local 14309 | \
	grep -v '^OK'


