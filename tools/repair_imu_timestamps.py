#!/usr/bin/env python3
"""Rewrite the timestamps of an imu*.csv recorded before the IMU timestamp fix.

Old dLidar builds gave each IMU sample `batch_base + idx * fixed_period`,
where batch_base was the real sensor stamp of the first sample in the batch.
Rows are in true acquisition order; only the timestamp columns are wrong.

Modes:
  anchored (default)  The first row of every batch still holds a real sensor
                      stamp. Those anchors are kept and the samples of each
                      batch are spread evenly up to the next anchor, which is
                      what the fixed recorder does live. timestampUnix becomes
                      timestamp + one constant offset (median of the original
                      anchor differences), so both columns share one clock.
                      The first timestamp is preserved; the last is
                      extrapolated with the mean sample spacing.
  uniform             Keep the first and last value of each column and space
                      all rows evenly in between.

Anchored falls back to uniform if the anchors are not strictly increasing.
The CSV format is kept: same header, single-space separator, measurement
columns copied verbatim, timestamps as integer nanoseconds. The original file
is never modified.
"""
import argparse
import os
import statistics
import sys
from collections import Counter

TS_COL, UNIX_COL = 7, 8


def uniform(first, last, n):
    if n == 1:
        return [first]
    return [first + (last - first) * i // (n - 1) for i in range(n)]


def anchored(ts, unix):
    n = len(ts)
    steps = [ts[i + 1] - ts[i] for i in range(n - 1)]
    if not steps:
        return None
    period = Counter(steps).most_common(1)[0][0]
    starts = [0] + [i + 1 for i, d in enumerate(steps) if d != period]
    # The old code restarted a batch on every sample while the stamp was 0
    # (its "first packet" test), so equal consecutive anchors are one group.
    starts = [s for k, s in enumerate(starts) if k == 0 or ts[s] != ts[starts[k - 1]]]
    anchors = [ts[i] for i in starts]
    if any(b <= a for a, b in zip(anchors, anchors[1:])):
        return None

    ends = anchors[1:]
    last_size = n - starts[-1]
    if len(starts) > 1:
        mean_dt = (anchors[-1] - anchors[0]) / starts[-1]
    else:
        mean_dt = period
    ends.append(anchors[-1] + round(mean_dt * last_size))

    bounds = starts + [n]
    new_ts = []
    for k, (start, end_t) in enumerate(zip(anchors, ends)):
        size = bounds[k + 1] - bounds[k]
        new_ts.extend(start + (end_t - start) * i // size for i in range(size))

    offset = round(statistics.median(unix[i] - ts[i] for i in starts))
    new_unix = [t + offset for t in new_ts]
    return new_ts, new_unix, period, len(starts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="original imu*.csv")
    ap.add_argument("-o", "--output", help="output path (default: <input>_fixed.csv)")
    ap.add_argument("--mode", choices=["anchored", "uniform"], default="anchored")
    ap.add_argument("--force", action="store_true", help="overwrite an existing output file")
    args = ap.parse_args()

    out_path = args.output or os.path.splitext(args.input)[0] + "_fixed.csv"
    if os.path.abspath(out_path) == os.path.abspath(args.input):
        sys.exit("error: output would overwrite the input")
    if os.path.exists(out_path) and not args.force:
        sys.exit(f"error: {out_path} exists (use --force to overwrite it)")

    with open(args.input, newline="") as f:
        header = f.readline()
        rows = [line.split() for line in f if line.strip()]
    cols = header.split()
    if len(cols) <= UNIX_COL or cols[TS_COL] != "timestamp" or cols[UNIX_COL] != "timestampUnix":
        sys.exit(f"error: unexpected header: {header.strip()}")
    if len(rows) < 2:
        sys.exit("error: need at least two IMU rows")

    ts = [int(float(r[TS_COL])) for r in rows]
    unix = [int(float(r[UNIX_COL])) for r in rows]

    result = anchored(ts, unix) if args.mode == "anchored" else None
    if result:
        new_ts, new_unix, period, batches = result
        print(f"anchored: {batches} batches, old fixed step {period / 1e6:.3f} ms")
    else:
        if args.mode == "anchored":
            print("anchored model not applicable (anchors not increasing), using uniform")
        new_ts = uniform(ts[0], ts[-1], len(ts))
        new_unix = uniform(unix[0], unix[-1], len(unix))

    with open(out_path, "w", newline="") as f:
        f.write(header)
        for r, t, u in zip(rows, new_ts, new_unix):
            r = r[:]
            r[TS_COL], r[UNIX_COL] = str(t), str(u)
            f.write(" ".join(r) + "\n")
    print(f"wrote {len(rows)} rows to {out_path}")


if __name__ == "__main__":
    main()
