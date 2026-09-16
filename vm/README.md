# Recording from an Apple Silicon Mac

The Xsens SDK has no macOS build and no ARM build, so on an Apple Silicon Mac
the only way to open the Awinda dongle is an emulated x86_64 guest. This
directory builds one: QEMU running Ubuntu 22.04 x86_64 with the dongle handed
through over USB.

Apple Silicon cannot virtualise x86_64, so every guest instruction is
translated by QEMU's TCG. That is much slower than native, and it does not
matter here -- 17 sensors at 60 Hz is about 50 kB/s.

**Measured on an M4 Pro (24 GB):** all 17 sensors connected, a sustained
60.1 Hz per sensor across a 20 s take, worst-case packet loss 0.166 %.

## Why USB passthrough is clean here

The dongle is an FTDI FT232R wearing Xsens' own USB vendor ID,
`0x2639:0x0102` (product code `AW-DNG2-ANT`). That single fact decides both
ends of the problem:

- **macOS ignores it.** Apple's `AppleUSBFTDI` driver matches only FTDI's own
  vendor ID `0x0403`, so nothing on the host claims the device and QEMU can
  take it without a fight. (It also means the dongle will never appear as
  `/dev/cu.usbserial-*` on macOS, so there is no native shortcut.)
- **Linux claims it immediately.** `0x2639:0x0102` has been in the in-tree
  `ftdi_sio` ID table since kernel 3.18, so the guest binds it to
  `/dev/ttyUSB0` with no driver work.

## Setup

    brew install qemu
    ./fetch.sh          # Ubuntu cloud image + MT Software Suite (~900 MB)
    ./make-seed.sh      # cloud-init seed
    ./prepare-disk.sh   # 24 GB working disk from the cloud image
    ./run-vm.sh         # boots; serial console on stdout

First boot takes a few minutes under emulation. Once sshd answers on port
2222, install the key and provision the guest:

    ./install-key.exp
    scp -P 2222 -i ~/.ssh/xsens_vm downloads/mtss-linux-x64-2022.2.tar.gz \
        provision.sh ../*.py xsens@127.0.0.1:~/
    ./vssh 'bash ~/provision.sh ~/mtss-linux-x64-2022.2.tar.gz'
    ./vssh 'pip3 install --user "numpy<2"'    # the SDK needs the numpy 1.x ABI

Then record exactly as you would on Windows:

    ./vssh 'cd ~ && python3 -u record_9axis.py --subject S01 --trial sprint_01 --seconds 20'
    scp -r -P 2222 -i ~/.ssh/xsens_vm xsens@127.0.0.1:~/data ./recovered/

Use `python3 -u`. Output is piped over ssh, so without unbuffered mode a run
you interrupt loses everything still sitting in Python's stdout buffer.

## Gotchas worth remembering

- **Verify the cloud image download.** A truncated qcow2 still has a valid
  header; the guest just fails to find its root filesystem at the offset where
  the data stopped. `fetch.sh` runs `qemu-img check` for this reason.
- **`ftdi_sio` is missing from the stock cloud kernel.** `provision.sh`
  installs `linux-modules-extra-$(uname -r)` and pins the module in
  `/etc/modules`.
- **`libusb_kernel_driver_active: -5 [NOT_FOUND]`** spam from QEMU on macOS is
  harmless -- libusb has no kernel-driver detach on macOS, and there is no
  driver to detach anyway.
- **"Only 0 sensors connected" is almost never the VM's fault.** The MTws join
  within about two seconds of being switched on. If none appear, they are
  simply off. What used to switch them off was our own teardown: dropping the
  master's radio powers every MTw down, so each take left the next one with
  nothing to talk to. `record_9axis.py` now leaves the radio up by default
  (`--release-radio` to power the sensors down when you're finished).
- The guest login is `xsens` / `xsens` (see `seed/user-data`). It is reachable
  only through the forwarded localhost port, but change it if that bothers you.
