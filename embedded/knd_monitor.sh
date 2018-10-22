#!/bin/bash
# KND keepalive script (loads KND config, restarts KND if it stops).
# Copyright (C)2011-2014 Mike Bourgeous.  Released under AGPLv3 in 2018.

# Note: limit size to <= 16000 characters for conspy compatibility
stty rows 152 cols 105

echo "Starting..."

rmmod gspca_kinect > /dev/null 2>&1

. /etc/nitrogenlogic/knd/knd.conf
export KND_SAVEDIR

while true; do nice -n 1 /usr/local/bin/knd 2>&1; sleep 1; done
