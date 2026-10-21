#!/usr/bin/env bash
# Replays the same packets through an older dLidar commit and the working
# tree, then checks that the standard recording content is identical and the
# newer additions are consistent (tools/verify_recording_format.py).
#
#   tools/compare_recording_formats.sh [OLD_REV] [PACKETS_BIN] [SECONDS] [OUT_DIR]
#
# Defaults: OLD_REV d68191b (last recording format 1 commit),
# PACKETS_BIN ~/PointCloudDump/packets0001.bin, SECONDS 1e9 (whole log, up to
# the 800-chunk limit), OUT_DIR a new temp directory. Writes OUT_DIR/old and
# OUT_DIR/new. Run `cmake --build build` first (for LASzip). Set PYTHON to an
# interpreter that has laspy[lazrs] and numpy (default python3).
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
old_rev=${1:-d68191b}
packets=${2:-$HOME/PointCloudDump/packets0001.bin}
seconds=${3:-1e9}
out=${4:-$(mktemp -d "${TMPDIR:-/tmp}/dlidar_formats_XXXXXX")}
python=${PYTHON:-python3}

work=$(mktemp -d "${TMPDIR:-/tmp}/dlidar_build_XXXXXX")
trap 'rm -rf "$work"' EXIT

mkdir -p "$work/old" "$out/old" "$out/new"
if [ -n "$(find "$out/old" "$out/new" -mindepth 1 -print -quit)" ]; then
    echo "error: $out/old and $out/new must be empty" >&2
    exit 1
fi
git -C "$repo" archive "$old_rev" include | tar -x -C "$work/old"

laszip_lib="$repo/build/3rd/lib"
build() { # include_dirs output
    g++ -O2 -std=c++17 -ffp-contract=off $1 -I"$repo/3rd/include" -I"$repo/3rd" "$repo/tools/replay_capture.cpp" \
        -L"$laszip_lib" -llaszip -Wl,-rpath,"$laszip_lib" -o "$2"
}
echo "building replay against $old_rev and the working tree ..."
build "-I$work/old/include" "$work/replay_old"
build "-I$repo/include -I$repo/src" "$work/replay_new"

echo "replaying $packets ($seconds s) ..."
"$work/replay_old" "$packets" "$out/old" "$seconds" > "$out/old.log" 2>&1
"$work/replay_new" "$packets" "$out/new" "$seconds" > "$out/new.log" 2>&1

"$python" -I "$repo/tools/verify_recording_format.py" "$out/old" "$out/new"
echo "recordings: $out/old (rev $old_rev), $out/new (working tree)"
