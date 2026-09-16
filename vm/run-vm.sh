#!/bin/bash
# Boot the x86_64 Ubuntu guest that runs the Xsens Linux SDK.
#
# Apple Silicon can't virtualise x86_64, so this is full TCG emulation. That is
# fine here: 17 sensors at 60 Hz is roughly 50 kB/s, nowhere near the limit.
#
# The Awinda dongle is handed to the guest by USB id rather than by bus/port,
# so replugging into a different socket doesn't break the mapping. macOS has no
# driver bound to VID 0x2639, so the device is free for QEMU to claim.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISK="$HERE/disk.qcow2"
SEED="$HERE/seed.iso"

AWINDA_VID=0x2639
AWINDA_PID=0x0102
SSH_PORT=2222

if [[ ! -f "$DISK" ]]; then
    echo "error: $DISK missing -- run prepare-disk.sh first" >&2
    exit 1
fi

exec qemu-system-x86_64 \
    -machine q35 \
    -cpu qemu64 \
    -smp 4 \
    -m 6144 \
    -drive "file=$DISK,if=virtio,format=qcow2" \
    -drive "file=$SEED,if=virtio,format=raw,readonly=on" \
    -netdev "user,id=net0,hostfwd=tcp::$SSH_PORT-:22" \
    -device virtio-net-pci,netdev=net0 \
    -device qemu-xhci,id=xhci \
    -device "usb-host,bus=xhci.0,vendorid=$AWINDA_VID,productid=$AWINDA_PID,id=awinda" \
    -display none \
    -serial mon:stdio \
    "$@"
