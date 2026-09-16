#!/bin/bash
# Download the two large inputs: the Ubuntu x86_64 cloud image and the Xsens
# Linux SDK. Both land in downloads/ and are left out of git.
#
# Note the integrity check on the cloud image. A partial download still has a
# valid qcow2 header, and the only symptom is the guest failing to find its root
# filesystem at whatever offset the data ran out -- so verify, don't assume.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DL="$HERE/downloads"
mkdir -p "$DL"

CLOUDIMG_URL=https://cloud-images.ubuntu.com/releases/22.04/release/ubuntu-22.04-server-cloudimg-amd64.img
SDK_URL=https://www.xsens.com/hubfs/Downloads/Software/MTSS/Releases/2022.2.0-stable-Awinda/MT_Software_Suite_linux-x64_2022.2_b7381_r124627.tar.gz

CLOUDIMG="$DL/ubuntu-2204-cloudimg-amd64.img"
SDK="$DL/mtss-linux-x64-2022.2.tar.gz"

if [[ -f "$CLOUDIMG" ]] && qemu-img check "$CLOUDIMG" >/dev/null 2>&1; then
    echo "cloud image already present and valid"
else
    echo "Downloading Ubuntu 22.04 cloud image (~700 MB)..."
    curl -fL --retry 3 -o "$CLOUDIMG.part" "$CLOUDIMG_URL"
    mv "$CLOUDIMG.part" "$CLOUDIMG"
    qemu-img check "$CLOUDIMG"
fi

if [[ -f "$SDK" ]]; then
    echo "SDK tarball already present"
else
    echo "Downloading MT Software Suite 2022.2 for Linux (~170 MB)..."
    curl -fL --retry 3 -o "$SDK.part" "$SDK_URL"
    mv "$SDK.part" "$SDK"
fi
tar tzf "$SDK" >/dev/null && echo "SDK tarball is readable"

echo
echo "Ready. Next: ./make-seed.sh && ./prepare-disk.sh && ./run-vm.sh"
