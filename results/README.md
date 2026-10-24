# Results

Benchmark summaries, plots and tables for the report.

Rules:

- Every benchmark result folder must include a `machine_info.json` written by
  `scripts/machine_info.sh` (set `HP_MACHINE_LABEL`), plus the engine's `hp::build_info()` string.
  Results without it are not used.
- Large raw output goes in `results/raw/`, which is git-ignored. Only small summaries, tables
  and plots are committed.
