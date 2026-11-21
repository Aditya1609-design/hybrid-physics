#!/usr/bin/env bash
# Local CI: records the machine, builds and tests each preset with warnings as errors,
# cross-checks recorded trajectories in Python (when .venv has numpy), and checks that no
# build output, datasets or model files are tracked by git.
# Usage: scripts/ci.sh [--clean] [--release-only] [--with-debug] [--with-viewer]
#   --with-viewer  also open the viewer on every scenario in each preset (needs a display;
#                  run it before a phase review)
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

clean=0
release_only=0
with_debug=0
with_viewer=0
for arg in "$@"; do
    case "$arg" in
        --clean) clean=1 ;;
        --release-only) release_only=1 ;;
        --with-debug) with_debug=1 ;;
        --with-viewer) with_viewer=1 ;;
        -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
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
            build/*|data/*|results/raw/*|*.pt|*.pth|*.onnx|*.bin|*.npz|*.hptraj)
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

# Runs hp_view on every scenario for a few frames: it must exit cleanly (sanitizer builds
# abort on any error) and end on the same state hash as hp_run from the same build.
viewer_smoke() {
    local p="$1" frames=30 bad=0 f out ref
    local view="build/$p/apps/viewer/hp_view" run="build/$p/apps/headless_runner/hp_run"
    if [[ ! -x "$view" ]]; then
        echo "hp_view is not built for $p (configured with HP_BUILD_VIEWER=OFF?)"
        return 1
    fi
    for f in scenarios/*.json; do
        if ! out=$("$view" "$f" --frames "$frames" 2>&1); then
            echo "$f: hp_view failed"
            echo "$out" | tail -20
            bad=1
            continue
        fi
        ref=$("$run" "$f" --frames "$frames" 2>/dev/null | grep "final state hash")
        if [[ "$(grep "final state hash" <<<"$out")" != "$ref" ]]; then
            echo "$f: final state differs from hp_run"
            bad=1
        else
            echo "$f: ok"
        fi
    done
    return $bad
}

# Records two runs with hp_run and checks each file against hp_run's own output in Python
# (ml/tools/check_trajectory.py): a realtime run with contact and a short reference run.
trajectory_check() {
    local p="$1" bad=0
    local run="build/$p/apps/headless_runner/hp_run" py=".venv/bin/python"
    "$py" ml/tools/check_trajectory.py --hp-run "$run" scenarios/cube_drop.json || bad=1
    "$py" ml/tools/check_trajectory.py --hp-run "$run" scenarios/cantilever_beam.json \
        --preset reference --frames 60 || bad=1
    return $bad
}

# The Python check needs the project venv with numpy; without it the step is skipped, not failed.
have_python=0
if [[ -x .venv/bin/python ]] && .venv/bin/python -c "import numpy" >/dev/null 2>&1; then
    have_python=1
fi

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
    if (( have_python )); then
        step "trajectory $p" trajectory_check "$p"
    else
        echo
        echo "==> trajectory $p: skipped (no .venv with numpy; see ml/requirements.txt)"
    fi
    if (( with_viewer )); then
        step "viewer $p" viewer_smoke "$p"
    fi
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
