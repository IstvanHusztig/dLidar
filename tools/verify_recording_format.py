#!/usr/bin/env python3
"""Compares a format 1 and a format 2 recording made from the same packets
(tools/compare_recording_formats.sh) and checks the format 2 additions.

    python3 tools/verify_recording_format.py OLD_DIR NEW_DIR

Each directory holds one recording with session number 0001. Checks: the
standard LAZ content, imu0001.csv and packets0001.bin are identical; old
timing keys are kept; no new .csv/.las/.laz; the extra bytes have the
documented names/types and agree with xyz and packetstate0001.tsv; the
sidecar has one row per point packet. Needs laspy with a LAZ backend
(pip install "laspy[lazrs]" numpy). Exit status 0 if every check passes.
"""
import sys, os, json, filecmp
import numpy as np, laspy

old_dir, new_dir = sys.argv[1], sys.argv[2]
ok = True
def check(cond, msg):
    global ok
    print(("PASS " if cond else "FAIL ") + msg)
    ok &= bool(cond)

print("== 1. same standard content ==")
o = laspy.read(os.path.join(old_dir, "lidar0001.laz"))
n = laspy.read(os.path.join(new_dir, "lidar0001.laz"))
ho, hn = o.header, n.header
print(f"old: LAS {ho.version} fmt {ho.point_format.id} reclen {ho.point_format.size} legacy count {ho.point_count} scale {ho.scales} offset {ho.offsets} gen '{ho.generating_software}'")
print(f"new: LAS {hn.version} fmt {hn.point_format.id} reclen {hn.point_format.size} legacy count {hn.point_count} scale {hn.scales} offset {hn.offsets} gen '{hn.generating_software}'")
check(str(hn.version) == "1.2" and hn.point_format.id == 1, "new is LAS 1.2 point format 1")
check(np.array_equal(ho.scales, hn.scales) and np.array_equal(ho.offsets, hn.offsets), "same scale and offset")
check(ho.point_count == hn.point_count == len(o.points) == len(n.points), f"same point count {len(n.points)}")
# the legacy 32-bit count in the raw header
with open(os.path.join(new_dir, "lidar0001.laz"), "rb") as f:
    raw = f.read(227)
legacy = int.from_bytes(raw[107:111], "little")
check(legacy == len(n.points), f"legacy number_of_point_records filled ({legacy})")
check(np.array_equal(ho.mins, hn.mins) and np.array_equal(ho.maxs, hn.maxs), "same header bounds")
for dim in ["X", "Y", "Z", "gps_time", "intensity", "user_data", "classification", "return_number",
            "number_of_returns", "point_source_id", "scan_angle_rank", "scan_direction_flag", "edge_of_flight_line"]:
    a, b = np.asarray(o[dim]), np.asarray(n[dim])
    check(np.array_equal(a, b), f"{dim} identical point for point")
check(np.all(np.asarray(n.user_data) == 1), "user_data == 1 everywhere")
check(filecmp.cmp(os.path.join(old_dir, "imu0001.csv"), os.path.join(new_dir, "imu0001.csv"), shallow=False), "imu0001.csv byte-identical")
check(filecmp.cmp(os.path.join(old_dir, "packets0001.bin"), os.path.join(new_dir, "packets0001.bin"), shallow=False), "packets0001.bin byte-identical")
jo = json.load(open(os.path.join(old_dir, "timing0001.json")))
jn = json.load(open(os.path.join(new_dir, "timing0001.json")))
check(all(k in jn and jn[k] == v for k, v in jo.items()), "timing0001.json: every old key kept with the same value")
print("     added keys:", {k: jn[k] for k in jn if k not in jo})
new_files = sorted(set(os.listdir(new_dir)) - set(os.listdir(old_dir)))
print("     files only in new:", new_files)
check(not any(f.lower().endswith((".csv", ".las", ".laz")) for f in new_files), "no new .csv/.las/.laz file")

print("== 2. extra bytes ==")
eb = [(d.name, str(d.dtype), d.description) for d in hn.point_format.extra_dimensions]
for e in eb:
    print("     ", e)
check([e[0] for e in eb] == ["range", "raw_range", "sample_index", "packet_seq", "alpha", "theta"], "extra dimension names")
check([e[1] for e in eb] == ["float32", "uint16", "uint16", "uint32", "float32", "float32"], "extra dimension types")
rng = np.asarray(n["range"]); raw_r = np.asarray(n["raw_range"]); j = np.asarray(n["sample_index"])
seq = np.asarray(n["packet_seq"]); alpha = np.asarray(n["alpha"]); theta = np.asarray(n["theta"])
xyz = np.vstack([n.x, n.y, n.z]).T
norm = np.linalg.norm(xyz, axis=1)

st = np.genfromtxt(os.path.join(new_dir, "packetstate0001.tsv"), delimiter="\t", names=True, dtype=None, encoding=None)
print(f"     packetstate rows: {len(st)}, columns: {len(st.dtype.names)}")
ab = float(np.max(np.hypot(st["a_axis_dist"], st["b_axis_dist"])))
d = norm - rng
print(f"     |xyz| - range: min {d.min():.6f} max {d.max():.6f} m (offset bound {ab:.6f} m + 1e-4 quantization)")
check(np.all(np.abs(d) <= ab + 2e-4), "range ≈ |xyz| within the a/b axis offsets")
check(j.min() >= 0 and j.max() <= 299, f"sample_index in 0..299 (min {j.min()} max {j.max()})")
print(f"     range {rng.min():.3f}..{rng.max():.3f} m, raw_range {raw_r.min()}..{raw_r.max()} mm, alpha {alpha.min():.3f}..{alpha.max():.3f}, theta {theta.min():.3f}..{theta.max():.3f} rad")

print("== 3. sidecar ==")
pk = np.fromfile(os.path.join(new_dir, "packets0001.bin"), dtype=np.dtype([("mono", "<i8"), ("unix", "<i8"), ("sens", "<i8"), ("seq", "<u4"), ("type", "<u4"), ("tinc", "<f4"), ("pn", "<u4")]), offset=8)
ptk = pk[pk["type"] == 102]
check(len(st) == len(ptk), f"one row per point packet ({len(st)} rows, {len(ptk)} point packets in the packet log)")
check(np.array_equal(st["seq"].astype(np.uint32), ptk["seq"]) and np.array_equal(st["l2_stamp_ns"], ptk["sens"]), "seq and L2 stamp match the packet log row by row")
zero = jn["zero_l2_ns"]
check(np.array_equal(st["stamp_ns"], st["l2_stamp_ns"] - zero), "stamp_ns = l2_stamp_ns - zero_l2_ns")
# points come packet by packet: split the LAZ where sample_index restarts or seq changes
brk = np.flatnonzero((np.diff(j.astype(np.int64)) <= 0) | (np.diff(seq.astype(np.int64)) != 0)) + 1
starts = np.concatenate([[0], brk])
pkt_of_point = np.repeat(np.arange(len(starts)), np.diff(np.concatenate([starts, [len(j)]])))
# rows that contributed points, in order: match by seq walking forward
rows = np.empty(len(starts), dtype=np.int64)
r = 0
seqs = st["seq"]
for g, s0 in enumerate(starts):
    while seqs[r] != seq[s0]:
        r += 1
    rows[g] = r
    r += 1
row_of_point = rows[pkt_of_point]
check(np.array_equal(seq, seqs[row_of_point].astype(np.uint32)), "packet_seq matches the packetstate row of each point")
tinc = st["time_increment"].astype(np.float32).astype(np.float64)
expect = st["stamp_ns"][row_of_point].astype(np.float64) / 1e9 + j.astype(np.float64) * tinc[row_of_point]
gt = np.asarray(n.gps_time)
check(np.array_equal(gt, expect), f"gps_time == stamp_ns/1e9 + sample_index*time_increment for every point (max diff {np.abs(gt-expect).max():.3g} s)")
first = j[starts] == 0
check(np.array_equal(gt[starts][first], st["stamp_ns"][rows][first] / 1e9), f"first point of each packet (j=0, {first.sum()} packets) has gps_time == stamp_ns/1e9")
rs = st["range_scale"].astype(np.float32)[row_of_point]; rb = st["range_bias"].astype(np.float32)[row_of_point]
check(np.array_equal(rng, rs * (raw_r.astype(np.float32) + rb)), "range == range_scale*(raw_range+range_bias) in float32, per packet")
gaps = np.diff(st["seq"].astype(np.int64)) % 1024
gaps_log = np.diff(ptk["seq"].astype(np.int64)) % 1024
print(f"     seq steps (mod 1024) in packetstate: {dict(zip(*np.unique(gaps, return_counts=True)))}")
check(np.array_equal(gaps, gaps_log), f"seq gaps match the packet log: {int(np.sum(gaps - 1))} lost point packets")
print("ALL PASSED" if ok else "SOME CHECKS FAILED")
sys.exit(0 if ok else 1)
