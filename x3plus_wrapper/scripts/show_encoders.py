#!/usr/bin/env python3
"""
show_encoders.py
----------------
Simple utility to print raw encoder counts from Rosmaster_Lib and the
per-interval differences (ticks). Useful for calibration and inspection.

Usage:
    python show_encoders.py --interval 0.1 --samples 100

Options:
    --interval FLOAT   Sampling interval in seconds (default: 0.1)
    --samples INT      Number of samples to take (default: run until Ctrl-C)
    --bits INT         Encoder bit-width for wrap-around correction (0 = disabled)
    --csv PATH         If provided, write CSV output to this file

"""
import time
import argparse
from Rosmaster_Lib import Rosmaster


def diff_with_wrap(new, old, bits):
    """Compute difference taking into account encoder wrap-around if bits>0."""
    if bits <= 0:
        return new - old
    mod = 1 << bits
    half = mod // 2
    diff = (new - old + half) % mod - half
    return int(diff)


def main(interval, samples, bits, csv_path):
    car = Rosmaster()
    try:
        # some Rosmaster clients create a receive thread to update sensors
        car.create_receive_threading()
    except Exception:
        # not critical if unavailable
        pass

    prev = None
    count = 0

    csv_file = None
    if csv_path:
        csv_file = open(csv_path, "w", buffering=1)
        csv_file.write("timestamp,front_left,rear_left,front_right,rear_right,dfl,drl,dfr,drr\n")

    print("Press Ctrl-C to stop. Sampling every {}s".format(interval))

    try:
        while True:
            enc = car.get_motor_encoder()
            # Rosmaster returns (front_left, rear_left, front_right, rear_right)
            fl, rl, fr, rr = map(int, enc)
            ts = time.time()

            if prev is None:
                dfl = drl = dfr = drr = 0
            else:
                dfl = diff_with_wrap(fl, prev[0], bits)
                drl = diff_with_wrap(rl, prev[1], bits)
                dfr = diff_with_wrap(fr, prev[2], bits)
                drr = diff_with_wrap(rr, prev[3], bits)

            line = "{:.3f},{},{},{},{},{},{},{},{}".format(ts, fl, rl, fr, rr, dfl, drl, dfr, drr)
            print(line)
            if csv_file:
                csv_file.write(line + "\n")

            prev = (fl, rl, fr, rr)
            count += 1
            if samples is not None and count >= samples:
                break
            time.sleep(interval)

    except KeyboardInterrupt:
        print("Interrupted by user, exiting...")
    finally:
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Print Rosmaster motor encoder counts and deltas.")
    parser.add_argument("--interval", type=float, default=0.1, help="Sampling interval in seconds")
    parser.add_argument("--samples", type=int, default=None, help="Number of samples to take (default: infinite)")
    parser.add_argument("--bits", type=int, default=0, help="Encoder bit-width for wrap-around correction (0 = disabled)")
    parser.add_argument("--csv", type=str, default=None, help="Write CSV to path")
    args = parser.parse_args()

    main(args.interval, args.samples, args.bits, args.csv)
