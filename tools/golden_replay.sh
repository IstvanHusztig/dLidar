#!/usr/bin/env bash
# Replays the same packet log through BASE_REV and the working tree and
# checks that every recording file and the console log are byte-identical.
# Used to prove that a refactor changes no output.
#
#   tools/golden_replay.sh [BASE_REV] [PACKETS_BIN] [SECONDS] [OUT_DIR]
#
# Defaults: BASE_REV 36ecead (golden base of the clean-architecture change),
# PACKETS_BIN ~/PointCloudDump/packets0001.bin, SECONDS 60 (1e9 = whole log),
# OUT_DIR a new temp directory, which gets base/ and new/.
# GOLDEN_COMPARE_ONLY=1 skips building and replaying and only compares OUT_DIR.
# BASE_REV is built once in a git worktree and cached in
# ${XDG_CACHE_HOME:-~/.cache}/dlidar_golden/<hash>. Both sides need the
# replay_capture CMake target writing bin/replay_capture
# (usage: replay_capture PACKETS out_dir [seconds]).
# Run on the Pi: aarch64 is where FMA contraction would differ; every target
# is built with -ffp-contract=off.
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
base_rev=${1:-36ecead}
packets=$(realpath "${2:-$HOME/PointCloudDump/packets0001.bin}")
seconds=${3:-60}
out=${4:-$(mktemp -d "${TMPDIR:-/tmp}/dlidar_golden_XXXXXX")}

hash=$(git -C "$repo" rev-parse --verify "$base_rev^{commit}")
cache=${XDG_CACHE_HOME:-$HOME/.cache}/dlidar_golden/$hash
base_bin=$cache/replay_capture

if [ -z "${GOLDEN_COMPARE_ONLY:-}" ]; then
if [ ! -x "$base_bin" ]; then
    echo "building replay_capture at $base_rev into $cache ..."
    rm -rf "$cache"
    mkdir -p "$(dirname "$cache")"
    git -C "$repo" worktree add --detach "$cache/src" "$hash" > /dev/null
    rmdir "$cache/src/3rd" 2> /dev/null || rm -rf "$cache/src/3rd"
    ln -s "$repo/3rd" "$cache/src/3rd" # LASzip submodule, same for both sides
    cmake -S "$cache/src" -B "$cache/build" -G Ninja > "$cache/configure.log"
    cmake --build "$cache/build" --target replay_capture > "$cache/build.log"
    # The worktree is only needed to build: keep the binary (its LASzip stays in
    # $cache/build) and drop the worktree.
    cp "$cache/src/bin/replay_capture" "$base_bin"
    git -C "$repo" worktree remove --force "$cache/src" 2> /dev/null || true
    git -C "$repo" worktree prune
fi
if [ ! -x "$base_bin" ]; then
    echo "error: base build did not produce $base_bin (see $cache/build.log)" >&2
    exit 1
fi

echo "building replay_capture of the working tree ..."
cmake --build "$repo/build" --target replay_capture > /dev/null

mkdir -p "$out/base/rec" "$out/new/rec"
if [ -n "$(find "$out/base/rec" "$out/new/rec" -mindepth 1 -print -quit)" ]; then
    echo "error: $out/base/rec and $out/new/rec must be empty" >&2
    exit 1
fi
# Relative output paths, so the two console logs can be compared verbatim.
echo "replaying $packets ($seconds s) ..."
(cd "$out/base" && "$base_bin" "$packets" rec "$seconds" > console.log 2>&1)
(cd "$out/new" && "$repo/bin/replay_capture" "$packets" rec "$seconds" > console.log 2>&1)
fi

status=0
for f in $(cd "$out/base/rec" && ls); do
    if [ ! -f "$out/new/rec/$f" ]; then
        echo "MISSING  $f"
        status=1
    elif cmp -s "$out/base/rec/$f" "$out/new/rec/$f"; then
        echo "same     $f ($(stat -c %s "$out/base/rec/$f") bytes)"
    else
        echo "DIFFERS  $f: $(cmp "$out/base/rec/$f" "$out/new/rec/$f" | head -1)"
        status=1
    fi
done
for f in $(cd "$out/new/rec" && ls); do
    [ -f "$out/base/rec/$f" ] || { echo "EXTRA    $f"; status=1; }
done
if diff -q "$out/base/console.log" "$out/new/console.log" > /dev/null; then
    echo "same     console log"
else
    echo "DIFFERS  console log (diff $out/base/console.log $out/new/console.log)"
    status=1
fi

if [ $status -eq 0 ]; then
    echo "identical (base $base_rev, output in $out)"
else
    echo "NOT identical (base $base_rev, output in $out)"
fi
exit $status
