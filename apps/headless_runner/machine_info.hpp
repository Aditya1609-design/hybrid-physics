#pragma once

#include <string>
#include <utility>
#include <vector>

// Describes the machine running hp_run, as key/value strings stored in trajectory headers.
// Keys follow scripts/machine_info.sh so recordings and benchmark results read the same way.
// Values that cannot be found are "unknown".
std::vector<std::pair<std::string, std::string>> collect_machine_info();

// Current UTC time as "YYYY-MM-DDTHH:MM:SSZ". Metadata only; never used by the physics.
std::string utc_timestamp();
