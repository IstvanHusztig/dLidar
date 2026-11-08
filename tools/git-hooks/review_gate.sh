#!/usr/bin/env bash
# Architecture review gate: a commit may be pushed, or merged into main, only
# when the architecture-reviewer agent approved it (.claude/agents/architecture-reviewer.md).
#
#   review_gate.sh approve SHA REPORT   record an approval (the agent calls this on PASS)
#   review_gate.sh check REV...         exit 1 if a revision lacks an approval
#   review_gate.sh check-merge          the git hooks: exit 1 if a branch being merged lacks one
#   review_gate.sh status [REV]         show the approval of REV (default HEAD)
#   review_gate.sh claude-hook          Claude Code PreToolUse hook (JSON on stdin)
#
# Approvals live in $(git rev-parse --git-common-dir)/arch-review/<sha>, untracked and
# shared by all worktrees. A merge commit needs approvals of the branches it merges
# (its parents after the first), not of itself. Bypass: git push --no-verify.
set -euo pipefail

MAIN=main
dir="$(git rev-parse --path-format=absolute --git-common-dir)/arch-review"

# The revisions that must be approved for REV: itself, or for a merge its merged parents.
required() {
    local sha parents
    sha=$(git rev-parse --verify --quiet "$1^{commit}") || { echo "unknown revision: $1" >&2; return 1; }
    parents=$(git rev-list --parents -n 1 "$sha" | cut -d' ' -f3-)
    if [ -n "$parents" ]; then echo "$parents" | tr ' ' '\n'; else echo "$sha"; fi
}

check() {
    local missing=0 rev sha
    for rev in "$@"; do
        for sha in $(required "$rev"); do
            if [ ! -f "$dir/$sha" ]; then
                echo "architecture review missing: $(git log -1 --format='%h %s' "$sha")" >&2
                missing=1
            fi
        done
    done
    if [ "$missing" -ne 0 ]; then
        echo "Run the architecture-reviewer agent in Claude Code on this branch first (or git push --no-verify)." >&2
        return 1
    fi
}

# The commits a merge in progress merges. git merge writes MERGE_HEAD only when it
# stops (a conflict, --no-commit), after pre-merge-commit has run, so an automatic
# merge is told by GIT_REFLOG_ACTION ("merge <branch>...", git merge sets it), a pull
# by FETCH_HEAD.
merge_heads() {
    local f arg
    f=$(git rev-parse --git-path MERGE_HEAD)
    if [ -f "$f" ]; then cat "$f"; return; fi
    case "${GIT_REFLOG_ACTION:-}" in
    "merge "*)
        for arg in ${GIT_REFLOG_ACTION#merge }; do
            case "$arg" in -*) continue ;; esac
            git rev-parse --verify --quiet "$arg^{commit}" || echo "unresolved:$arg"
        done
        ;;
    pull*)
        f=$(git rev-parse --git-path FETCH_HEAD)
        [ -f "$f" ] && grep -v $'\tnot-for-merge\t' "$f" | cut -f1
        ;;
    esac
}

case "${1:-}" in
check-merge)
    heads=$(merge_heads)
    if [ -z "$heads" ] || grep -q '^unresolved:' <<<"$heads"; then
        echo "architecture review: cannot tell which commits this merge brings in (${heads:-nothing found}); refusing." >&2
        echo "Merge a branch by name (git merge BRANCH), or bypass with --no-verify." >&2
        exit 1
    fi
    # shellcheck disable=SC2086
    check $heads
    ;;
approve)
    sha=$(git rev-parse --verify "$2^{commit}")
    mkdir -p "$dir"
    { echo "approved $(date -Iseconds) $(git log -1 --format='%h %s' "$sha")"; cat "$3"; } > "$dir/$sha"
    echo "approved $sha"
    ;;
check)
    shift
    check "$@"
    ;;
status)
    sha=$(git rev-parse --verify "${2:-HEAD}^{commit}")
    if [ -f "$dir/$sha" ]; then head -1 "$dir/$sha"; else echo "not approved: $sha"; exit 1; fi
    ;;
claude-hook)
    # Blocks `git push` and `git merge` into main in Claude's Bash calls unless approved.
    revs=$(python3 -c '
import json, shlex, subprocess, sys
cmd = json.load(sys.stdin).get("tool_input", {}).get("command", "")
branch = subprocess.run(["git", "branch", "--show-current"], capture_output=True, text=True).stdout.strip()
with_value = {"-m", "-F", "-s", "-X", "--strategy", "--strategy-option", "--file", "--message", "--into-name", "--cleanup", "-S", "--gpg-sign"}
for part in cmd.replace("&&", ";").replace("||", ";").split(";"):
    try:
        w = shlex.split(part)
    except ValueError:
        continue
    while w and "=" in w[0]:
        w = w[1:]
    if len(w) < 2 or w[0] != "git":
        continue
    args, skip = [], False
    for a in w[2:]:
        if skip:
            skip = False
        elif a in with_value:
            skip = True
        elif not a.startswith("-"):
            args.append(a)
    if w[1] in ("checkout", "switch") and args:
        branch = args[0]  # the branch a later git merge in this command runs on
    if w[1] == "push" and "--delete" not in w and "-d" not in w:
        src = args[1].split(":")[0].lstrip("+") if len(args) > 1 else "HEAD"
        print(src or "HEAD")
    elif w[1] == "merge" and not {"--abort", "--continue", "--quit"} & set(w):
        if branch == sys.argv[1]:
            print("\n".join(args))
' "$MAIN")
    [ -z "$revs" ] && exit 0
    # shellcheck disable=SC2086
    if ! out=$(check $revs 2>&1); then
        python3 -c 'import json, sys; print(json.dumps({"hookSpecificOutput": {"hookEventName": "PreToolUse",
            "permissionDecision": "deny", "permissionDecisionReason": sys.argv[1] +
            "\nLaunch the architecture-reviewer agent on these commits; push or merge once it records PASS."}}))' "$out"
    fi
    ;;
*)
    sed -n '2,13p' "$0"
    exit 2
    ;;
esac
