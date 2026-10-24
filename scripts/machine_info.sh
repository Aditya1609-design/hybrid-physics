#!/usr/bin/env bash
# Prints the machine and toolchain as JSON. Every benchmark result stores this output so
# numbers from different machines are never mixed up.
# Usage: scripts/machine_info.sh [-o out.json]   (set HP_MACHINE_LABEL to tag the machine)
set -euo pipefail

out=""
if [[ "${1:-}" == "-o" ]]; then
    out="${2:?missing output path}"
fi

root="$(cd "$(dirname "$0")/.." && pwd)"

# Escapes backslashes and quotes so tool output is safe inside JSON strings.
json_str() {
    local s="${1//\\/\\\\}"
    printf '"%s"' "${s//\"/\\\"}"
}

# Reads a sysctl key, or "unknown" when it is missing.
sys() { sysctl -n "$1" 2>/dev/null || echo unknown; }

first_line() { "$@" 2>/dev/null | head -n 1 || echo unknown; }

mem_bytes="$(sys hw.memsize)"
if [[ "$mem_bytes" =~ ^[0-9]+$ ]]; then
    mem_gb=$((mem_bytes / 1024 / 1024 / 1024))
else
    mem_gb=unknown
fi

commit="$(git -C "$root" rev-parse HEAD 2>/dev/null || echo unknown)"
branch="$(git -C "$root" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
if [[ -n "$(git -C "$root" status --porcelain 2>/dev/null)" ]]; then dirty=true; else dirty=false; fi

json=$(cat <<EOF
{
  "label": $(json_str "${HP_MACHINE_LABEL:-unlabeled}"),
  "timestamp_utc": $(json_str "$(date -u +%Y-%m-%dT%H:%M:%SZ)"),
  "chip": $(json_str "$(sys machdep.cpu.brand_string)"),
  "arch": $(json_str "$(uname -m)"),
  "cores_total": $(json_str "$(sys hw.physicalcpu)"),
  "cores_performance": $(json_str "$(sys hw.perflevel0.physicalcpu)"),
  "cores_efficiency": $(json_str "$(sys hw.perflevel1.physicalcpu)"),
  "memory_gb": $(json_str "$mem_gb"),
  "os": $(json_str "$(sw_vers -productName 2>/dev/null || uname -s) $(sw_vers -productVersion 2>/dev/null || uname -r)"),
  "os_build": $(json_str "$(sw_vers -buildVersion 2>/dev/null || echo unknown)"),
  "compiler": $(json_str "$(first_line c++ --version)"),
  "cmake": $(json_str "$(first_line cmake --version)"),
  "ninja": $(json_str "$(first_line ninja --version)"),
  "git_commit": $(json_str "$commit"),
  "git_branch": $(json_str "$branch"),
  "git_dirty": $dirty
}
EOF
)

if [[ -n "$out" ]]; then
    mkdir -p "$(dirname "$out")"
    printf '%s\n' "$json" > "$out"
fi
printf '%s\n' "$json"
