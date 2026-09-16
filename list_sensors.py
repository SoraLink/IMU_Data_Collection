"""
Connection test only -- no pipeline, no plotting.

Confirms:
  - the SDK can open the Awinda dongle's COM port (MT Manager must be closed)
  - the radio comes up and paired sensors reconnect
  - what string format XsDeviceId prints as, so SENSOR_MAP in pipeline.py
    can be filled in with the exact matching keys

Run it, put the sensors in view of the dongle, and watch the console.
Ctrl+C to stop.
"""

import time
import xsensdeviceapi as xda

RADIO_CHANNEL = 11  # change if your dongle uses a different channel


def find_master(retries=5, delay=1.0):
    control = xda.XsControl_construct()
    for attempt in range(retries):
        ports = xda.XsScanner_scanPorts()
        for i in range(ports.size()):
            p = ports[i]
            if p.deviceId().isWirelessMaster():
                return control, p
        print(f"  scan attempt {attempt + 1}/{retries}: no master found, retrying...")
        time.sleep(delay)
    raise RuntimeError(
        "No Awinda master found after retries. Is the dongle plugged in, and is "
        "MT Manager fully closed (it holds the port exclusively)?"
    )


def main():
    control, master = find_master()

    print(f"Found master on {master.portName()} @ {master.baudrate()} baud, "
          f"id={master.deviceId()}")

    if not control.openPort(master):
        raise RuntimeError("openPort failed -- is another program using the COM port?")

    dev = control.device(master.deviceId())
    dev.gotoConfig()
    dev.enableRadio(RADIO_CHANNEL)
    print(f"Radio on (channel {RADIO_CHANNEL}). Waiting for sensors to reconnect...")

    seen = set()
    try:
        while True:
            children = dev.children()
            for i in range(children.size()):
                child = children[i]
                did = str(child.deviceId())
                if did not in seen:
                    seen.add(did)
                    print(f"  connected: {did}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nTotal sensors seen: {len(seen)}")
        for did in sorted(seen):
            print(f"  {did}")
        dev.gotoConfig()
        dev.disableRadio()
        control.close()
        print("Closed.")


if __name__ == "__main__":
    main()
