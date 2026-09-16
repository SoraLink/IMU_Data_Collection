#!/bin/bash
# Turn the downloaded Ubuntu cloud image into this VM's working disk.
# Kept separate from run-vm.sh so re-running the VM never wipes the guest.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="$HERE/downloads/ubuntu-2204-cloudimg-amd64.img"
DISK="$HERE/disk.qcow2"
SIZE=24G

if [[ ! -f "$BASE" ]]; then
    echo "error: base image $BASE not found" >&2
    exit 1
fi

if [[ -f "$DISK" ]]; then
    echo "$DISK already exists -- refusing to overwrite an existing guest."
    echo "Delete it by hand first if you really want a fresh install."
    exit 1
fi

# Verify the base before copying -- a half-finished download still looks like a
# valid qcow2 header, and the failure only shows up as guest I/O errors at the
# offset where the data ran out.
echo "Checking base image..."
qemu-img check "$BASE" || { echo "base image is damaged; re-download it" >&2; exit 1; }

cp "$BASE" "$DISK"
qemu-img resize "$DISK" "$SIZE"

echo "Checking working disk..."
qemu-img check "$DISK" || { echo "copy is damaged" >&2; exit 1; }
qemu-img info "$DISK"
echo "Disk ready. Start the VM with ./run-vm.sh"
