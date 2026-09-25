#!/bin/sh
# Print live telemetry from the board. Ctrl-C to stop.
PORT=$(ls /dev/serial/by-id/usb-EBB42_standalone_* 2>/dev/null | head -1)
[ -n "$PORT" ] || { echo "board not found on USB"; exit 1; }
stty -F "$PORT" 115200 raw -echo
exec cat "$PORT"
