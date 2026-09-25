#!/bin/sh
# Flash the firmware over USB DFU.
#
# If the board is already running this firmware (shows up as /dev/ttyACM*),
# send it the 'b' command so it reboots into the ROM DFU bootloader itself.
# Otherwise put it in DFU mode manually: hold BOOT, tap RESET.
# `lsusb` should then show 0483:df11.
set -e
cd "$(dirname "$0")"

PORT=$(ls /dev/serial/by-id/usb-EBB42_standalone_* 2>/dev/null | head -1)
if [ -n "$PORT" ]; then
    echo "Asking $PORT to enter DFU bootloader..."
    stty -F "$PORT" 115200 raw -echo
    printf 'b' > "$PORT"
    i=0
    while ! dfu-util -l 2>/dev/null | grep -q 0483:df11; do
        i=$((i+1)); [ $i -gt 50 ] && { echo "DFU device did not appear"; exit 1; }
        sleep 0.2
    done
fi

dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D build/ebb42_heater.bin || true
echo "(dfu-util 'Error during download get_status' after leave is normal: the board has reset)"
