#!/bin/bash
# Copyright (C)2011 Mike Bourgeous.  Released under AGPLv3 in 2018.
# A script that uses the center of gravity feature to adjust light brightness
# based on hand position within a zone.  The hands zone looks like this:
# 
# xmin=-0.580 ymin=0.144 zmin=0.462 xmax=0.700 ymax=0.528 zmax=0.798 px_xmin=0
#   px_ymin=0 px_zmin=350 px_xmax=639 px_ymax=158 px_zmax=656 occupied=0 pop=0
#   maxpop=100962 xc=0.000 yc=0.000 zc=0.000 name="Hands"
# 
# The xc value is offset and scaled to the light value range (0-100), then sent
# to the automation controller.

echo Press Enter to stop this script.

(echo sub; read) | \
	nc localhost 14308 | \
	grep --line-buffered occupied=1 | \
	grep --line-buffered -o 'xc=[^ ]*' | \
	sed -ue 's/xc=\(.*\)/(\1 + 0.6) * 100.0/' | \
	bc | \
	sed -ue 's/^\([^.]*\)\..*/set 13,4,\1/' | \
	nc logic-controller.local 14309 | \
	grep -v '^OK'


