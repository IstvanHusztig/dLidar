#!/bin/sh
# Opens dLidar's Bluetooth pairing window for 2 minutes: a phone can then
# bond (Just Works, no PIN). Works for the service and for a dLidar started
# by hand; no sudo needed when it runs as your user (docs/manual.html,
# "Phone link"). The pairing button does the same (short press, 5 s hold).
#
#   tools/pair.sh            open the window
#   tools/pair.sh --forget   dLidar first removes every bonded phone on the
#                            Pi, then opens the window: for a phone that was
#                            unpaired on its side (the Pi cannot pair it
#                            again over its old bond), or a lost one
set -e
signal=USR1
if [ "$1" = "--forget" ]; then
    signal=USR2
elif [ -n "$1" ]; then
    echo "usage: tools/pair.sh [--forget]" >&2
    exit 2
fi
if pkill -"$signal" -x dLidar; then
    [ "$signal" = USR2 ] && echo "dLidar forgets every bonded phone (its log says how many)."
    echo "Pairing window opened for 2 minutes: connect with the app and tap Pair."
    echo "dLidar's log (journalctl -u dlidar -f) shows when a phone paired and when the window closes."
else
    echo "No dLidar is running." >&2
    exit 1
fi
