#!/bin/bash
# Test of zonelights script with knd as the backend

: ${HELLO_MESSAGE:="Hello"}

LOGIC_HOST=logic-controller.local
KINECT_HOST=localhost
KINECT_PORT=14308

# say_it "text" [params]
function say_it()
{
	if [ -e "$(which espeak)" ]; then
		TEXT="$1"
		shift
		echo "$TEXT" | espeak -v en-us > /dev/null 2>&1
	fi
}

# send_it < commands
function send_it()
{
	(cat; echo bye) | nc -q2 $LOGIC_HOST 14309
}

# set_val objid param value | send_it
function setv()
{
	echo "set $1,$2,$3\n"
}

# bright intensity(0-100) | send_it
function bright()
{
	: setv 13 4 $1
}

# scene preset#(0-3) | send_it
function scene()
{
	setv 17 2 $1
}

# onoff state(0-1) | send_it
function onoff()
{
	setv 12 0 $1
}


LAST_ROOM=""
HERE=1
while true; do
	sleep 0.125
	# eval is dangerous
	eval $(printf "zones\nbye\n" | nc -q2 $KINECT_HOST 14308 | grep 'occupied=1' | sort -r -b -t ' ' -k 14.5 -n | head -n 1 | grep -o 'name=.*"$')
	if [ "$name" = "${LAST_ROOM}" ]; then
		continue
	fi
	if [ "$ANNOUNCE_ZONE" != "" -a "$name" != "[none]" -a "$LAST_ROOM" != "$name" ]; then
		say_it "$name" -p 0 &
	fi
	case "$name" in
		Desk)
			echo "Work mode ($name)"
			(bright 100; scene 1) | send_it
			;;
		Couch*)
			echo "Movie mode ($name)"
			(bright 10; scene 2) | send_it
			;;
		Bedroom)
			echo "Sleep mode ($name)"
			(bright 50; scene 3) | send_it
			;;
		"[none]")
			;;
		Entry*)
			if [ "$name" = "Entry" -a "$LAST_ROOM" = "Entry2" ]; then
				echo "All lights off ($LAST_ROOM->$name)"
				(onoff 0) | send_it
				say_it "Goodbye"
				HERE=0
			else
				echo "All lights on ($LAST_ROOM->$name)"
				(bright 100; scene 0) | send_it
				if [ \( "$name" = "Entry2" -a "$LAST_ROOM" = "Entry" \) -o "$name" = "Entry" ]; then
					(onoff 1) | send_it
					if [ $HERE -eq 0 ]; then
						say_it "$HELLO_MESSAGE" -s 160
					fi
				fi
				HERE=1
			fi
			;;
		*)
			echo "All lights on ($name)"
			(bright 100; scene 0) | send_it
			;;
	esac
	LAST_ROOM=$name
done
