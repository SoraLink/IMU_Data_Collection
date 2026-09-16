#!/bin/bash
# Runs inside the Ubuntu guest. Installs the Xsens Linux SDK and the Python
# bindings, then checks the Awinda dongle actually arrived over USB passthrough.
#
# Usage:  ./provision.sh /path/to/mtss-linux-x64-2022.2.tar.gz

set -euo pipefail

TARBALL="${1:-$HOME/mtss-linux-x64-2022.2.tar.gz}"
WORK="$HOME/mtss"

banner() { printf '\n=== %s ===\n' "$1"; }

banner "System check"
echo "arch:   $(uname -m)   (must be x86_64 -- the SDK has no ARM build)"
echo "kernel: $(uname -r)"
echo "python: $(python3 -V)"
if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "error: not an x86_64 guest; the closed-source libxsensdeviceapi.so" \
         "cannot load here" >&2
    exit 1
fi

banner "Dependencies"
sudo apt-get update -qq
sudo apt-get install -y -qq \
    build-essential python3-pip python3-dev sharutils \
    libusb-1.0-0 usbutils >/dev/null
echo "installed"

banner "USB passthrough check"
if lsusb -d 2639: >/dev/null 2>&1; then
    lsusb -d 2639:
    echo "Awinda hardware is visible to the guest."
else
    echo "WARNING: no VID 2639 device on the guest USB bus."
    echo "The dongle was not passed through -- everything below will fail."
    lsusb || true
fi

banner "Serial driver binding"
# 0x2639:0x0102 has been in mainline ftdi_sio since 3.18, so this should be
# automatic; new_id is the fallback for anything not yet in the table.
sudo modprobe ftdi_sio || true
sleep 1
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    ls -l /dev/ttyUSB*
else
    echo "no /dev/ttyUSB* yet; registering the id explicitly"
    echo 2639 0102 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id >/dev/null || true
    sleep 1
    ls -l /dev/ttyUSB* 2>/dev/null || echo "still none -- check passthrough"
fi

banner "Unpacking the MT Software Suite"
rm -rf "$WORK" && mkdir -p "$WORK"
tar xzf "$TARBALL" -C "$WORK" --strip-components=1
ls "$WORK"

banner "Installing the SDK"
cd "$WORK"
# The installer is interactive; feed it the default prefix and accept.
sudo ./mtsdk_linux-x64_2022.2.sh <<'ANSWERS' || true
/usr/local/xsens
yes
ANSWERS
ls /usr/local/xsens 2>/dev/null || echo "(installer may need manual answers)"

banner "Installing the Python bindings"
WHEEL=$(find /usr/local/xsens "$WORK" -name 'xsensdeviceapi-*cp310*linux_x86_64.whl' 2>/dev/null | head -1)
if [[ -z "$WHEEL" ]]; then
    echo "cp310 wheel not found; searching all wheels"
    find /usr/local/xsens "$WORK" -name '*.whl' 2>/dev/null || true
    exit 1
fi
echo "wheel: $WHEEL"
pip3 install --user "$WHEEL"
pip3 install --user numpy matplotlib

banner "Import test"
python3 - <<'PY'
import xsensdeviceapi as xda
print("xsensdeviceapi imported OK")
print("version:", xda.xdaVersion() if hasattr(xda, "xdaVersion") else "n/a")
ports = xda.XsScanner_scanPorts()
print(f"scanPorts found {ports.size()} port(s)")
for i in range(ports.size()):
    p = ports[i]
    print(f"  {p.portName()} @ {p.baudrate()} baud  id={p.deviceId()}  "
          f"wireless_master={p.deviceId().isWirelessMaster()}")
PY

banner "Done"
echo "If scanPorts listed a wireless master, the pipeline is ready to run."
