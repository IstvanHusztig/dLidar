#!/bin/sh
# Opens dLidar's Bluetooth pairing window for 2 minutes: a phone can then
# bond (Just Works, no PIN). Works for the service and for a dLidar started
# by hand; no sudo needed when it runs as your user (docs/manual.html,
# "Phone link"). The pairing button will do the same.
#
#   tools/pair.sh            open the window
#   tools/pair.sh --forget   first remove every bonded phone on the Pi, then
#                            open the window: for a phone that was unpaired
#                            on its side (the Pi cannot pair it again over
#                            its old bond)
set -e
if [ "$1" = "--forget" ]; then
    for address in $(bluetoothctl devices Paired | awk '{print $2}'); do
        bluetoothctl remove "$address" > /dev/null && echo "Forgot $address."
    done
elif [ -n "$1" ]; then
    echo "usage: tools/pair.sh [--forget]" >&2
    exit 2
fi
if pkill -USR1 -x dLidar; then
    echo "Pairing window opened for 2 minutes: connect with the app and tap Pair."
    echo "dLidar's log (journalctl -u dlidar -f) shows when a phone paired and when the window closes."
else
    echo "No dLidar is running." >&2
    exit 1
fi
