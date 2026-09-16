#!/bin/bash
# Build the cloud-init NoCloud seed image from seed/.
# The volume label must be exactly CIDATA or cloud-init will not find it.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -f "$HERE/seed.iso"
hdiutil makehybrid -iso -joliet \
    -joliet-volume-name CIDATA \
    -default-volume-name CIDATA \
    -o "$HERE/seed.iso" "$HERE/seed" >/dev/null

echo "seed.iso built:"
ls -lh "$HERE/seed.iso"
