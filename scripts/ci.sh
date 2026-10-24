#!/usr/bin/env bash
# Local CI: records the machine, builds and tests each preset with warnings as errors,
# and checks that no build output, datasets or model files are tracked by git.
# Usage: scripts/ci.sh [--clean] [--release-only] [--with-debug]
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

clean=0
release_only=0
with_debug=0
for arg in "$@"; do
    case "$arg" in
        --clean) clean=1 ;;
        --release-only) release_only=1 ;;
        --with-debug) with_debug=1 ;;
        -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

presets=(release asan)
if (( release_only )); then
    presets=(release)
elif (( with_debug )); then
    presets+=(debug)
fi

# Largest tracked file allowed, in KB. Anything bigger belongs in data/ or results/raw/.
max_tracked_kb="${HP_MAX_TRACKED_KB:-1024}"

# A plain string rather than an array: empty arrays trip `set -u` in macOS's bash 3.2.
failures=""

# Runs one step, prints its duration, and records it as failed instead of stopping.
step() {
    local name="$1"; shift
    local start=$SECONDS
    echo
    echo "==> $name"
    if "$@"; then
        echo "--- $name: ok ($((SECONDS - start)) s)"
    else
        echo "--- $name: FAILED ($((SECONDS - start)) s)"
        failures="$failures [$name]"
    fi
}

# Fails if git tracks generated files or anything larger than the size limit.
check_tracked_files() {
    local bad=0 f size_kb
    while IFS= read -r -d '' f; do
        case "$f" in
            data/MANIFEST.md) ;;
            build/*|data/*|results/raw/*|*.pt|*.pth|*.onnx|*.bin|*.npz)
                echo "tracked file should be ignored: $f"; bad=1 ;;
        esac
        if [[ -f "$f" ]]; then
            size_kb=$(( $(wc -c < "$f") / 1024 ))
            if (( size_kb > max_tracked_kb )); then
                echo "tracked file too large (${size_kb} KB > ${max_tracked_kb} KB): $f"; bad=1
            fi
        fi
    done < <(git ls-files -z)
    return $bad
}

if (( clean )); then
    echo "==> removing build/"
    rm -rf build
fi

echo "==> machine"
scripts/machine_info.sh -o build/ci/machine_info.json

for p in "${presets[@]}"; do
    step "configure $p" cmake --preset "$p" -DHP_WARNINGS_AS_ERRORS=ON
    step "build $p" cmake --build --preset "$p"
    step "test $p" ctest --preset "$p"
done

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    step "tracked files" check_tracked_files
fi

echo
if [[ -z "$failures" ]]; then
    echo "CI PASS (${presets[*]})"
else
    echo "CI FAIL:$failures"
    exit 1
fi
