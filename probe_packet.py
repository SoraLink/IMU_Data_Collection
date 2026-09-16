"""What does an MTw packet actually contain, and can output config be set
on the master or on the individual MTw devices?"""

import time
import threading

import xsensdeviceapi as xda
from pipeline import SENSOR_MAP, RADIO_CHANNEL, DESIRED_RATE, find_master

CHECKS = [
    "containsPacketCounter", "containsSampleTimeFine",
    "containsOrientation", "containsCalibratedData",
    "containsCalibratedAcceleration", "containsCalibratedGyroscopeData",
    "containsCalibratedMagneticField", "containsRawData",
    "containsRawAcceleration", "containsRawGyroscopeData",
    "containsRawMagneticField", "containsFreeAcceleration",
    "containsCorrectedMagneticField", "containsOrientationIncrement",
]


class Probe(xda.XsCallback):
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.report = None

    def onLiveDataAvailable(self, dev, packet):
        if packet is None:
            return
        with self.lock:
            if self.report is not None:
                return
            r = {}
            for name in CHECKS:
                try:
                    r[name] = bool(getattr(packet, name)())
                except Exception as e:
                    r[name] = f"ERR {e}"
            vals = {}
            try:
                if packet.containsCalibratedAcceleration():
                    a = packet.calibratedAcceleration()
                    vals["acc"] = (a[0], a[1], a[2])
                if packet.containsCalibratedGyroscopeData():
                    g = packet.calibratedGyroscopeData()
                    vals["gyr"] = (g[0], g[1], g[2])
                if packet.containsCalibratedMagneticField():
                    m = packet.calibratedMagneticField()
                    vals["mag"] = (m[0], m[1], m[2])
            except Exception as e:
                vals["error"] = str(e)
            self.report = (r, vals)


def main():
    control, port = find_master()
    control.openPort(port)
    master = control.device(port.deviceId())
    master.gotoConfig()

    rates = master.supportedUpdateRates()
    rate = min([rates[i] for i in range(rates.size())],
               key=lambda r: abs(r - DESIRED_RATE))
    master.setUpdateRate(rate)

    try:
        oc = master.outputConfiguration()
        print(f"master.outputConfiguration() size = {oc.size()}")
        for i in range(oc.size()):
            print(f"   {oc[i].m_dataIdentifier:#06x} @ {oc[i].m_frequency} Hz")
    except Exception as e:
        print(f"master.outputConfiguration() raised: {e}")

    if master.isRadioEnabled():
        master.disableRadio()
    master.enableRadio(RADIO_CHANNEL)
    print("Radio on, waiting for sensors...")
    start, seen = time.time(), -1
    while master.children().size() < len(SENSOR_MAP) and time.time() - start < 60:
        n = master.children().size()
        if n != seen:
            seen = n
            print(f"  {n}/{len(SENSOR_MAP)}")
        time.sleep(0.3)
    print(f"{master.children().size()} sensors connected")
    if master.children().size() == 0:
        print("No sensors -- are they powered on? Aborting probe.")
        master.gotoConfig()
        master.disableRadio()
        control.close()
        return

    # Try setting output config on one MTw while still in config mode.
    kids = master.children()
    if kids.size():
        mtw = kids[0]
        print(f"\n--- probing MTw {mtw.deviceId()} in config mode ---")
        print(f"  canOutputConfiguration = {mtw.canOutputConfiguration()}")
        cfg = xda.XsOutputConfigurationArray()
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, rate))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, rate))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, rate))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, rate))
        ok = mtw.setOutputConfiguration(cfg)
        print(f"  mtw.setOutputConfiguration() = {ok}")
        if not ok:
            print(f"  lastResultText: {control.lastResultText()}")

    master.gotoMeasurement()

    probes = {}
    ids = control.deviceIds()
    for i in range(ids.size()):
        did = ids[i]
        if not did.isMtw():
            continue
        seg = SENSOR_MAP.get(str(did), str(did))
        p = Probe()
        control.device(did).addCallbackHandler(p)
        probes[seg] = p

    print(f"\nCollecting one packet from each of {len(probes)} sensors...")
    time.sleep(5)

    if not probes:
        print("No mapped MTw devices found.")
        master.gotoConfig()
        master.disableRadio()
        control.close()
        return

    seg = sorted(probes)[0]
    rep = probes[seg].report
    print(f"\n=== packet contents ({seg}) ===")
    if rep is None:
        print("  no packet received")
    else:
        flags, vals = rep
        for k, v in flags.items():
            print(f"  {k:<36} {v}")
        print("\n=== sample values ===")
        for k, v in vals.items():
            print(f"  {k}: {v}")

    got = sum(1 for p in probes.values() if p.report is not None)
    print(f"\n{got}/{len(probes)} sensors produced a packet")

    master.gotoConfig()
    master.disableRadio()
    control.close()
    print("Closed.")


if __name__ == "__main__":
    main()
