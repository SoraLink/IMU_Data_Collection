"""Validate the data path only: connect, stream, print a few quaternions.

No plotting, so any failure here is a pure SDK/wiring problem.
"""

import time
import pipeline


def main():
    control, master, callbacks = pipeline.connect()
    try:
        print("\nStreaming for 8 s...\n")
        deadline = time.time() + 8
        while time.time() < deadline:
            time.sleep(2.0)
            live = {s: cb.get() for s, cb in callbacks.items()}
            got = {s: q for s, q in live.items() if q is not None}
            print(f"--- {len(got)}/{len(callbacks)} segments reporting ---")
            for seg in sorted(got):
                q = got[seg]
                print(f"  {seg:<16} w={q[0]:+.3f} x={q[1]:+.3f} "
                      f"y={q[2]:+.3f} z={q[3]:+.3f}")
    finally:
        print("\nShutting down...")
        master.gotoConfig()
        master.disableRadio()
        control.close()
        print("Closed.")


if __name__ == "__main__":
    main()
