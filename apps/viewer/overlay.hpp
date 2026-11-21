#pragma once

#include <string>
#include <vector>

#include "core/colors.hpp"
#include "core/sim_driver.hpp"
#include "render.hpp"

namespace hp::viewer {

// What the user has switched on or off, shown in the overlay.
struct UiState {
    bool paused = false;
    DisplayMode display = DisplayMode::Solid;
    bool tet_edges = false;
    ColorMode color_mode = ColorMode::Owner;
    bool preview = false;   // display-only owner/region pattern instead of the real arrays
    bool follow = false;    // camera target follows the body centroid
    bool show_stats = true;
    bool show_help = false;
    std::string message;    // short notice (reload result and similar), cleared after a while
    double message_until = 0.0;
};

// Rendering numbers measured by the app, not the simulation.
struct RenderStats {
    int fps = 0;
    double steps_per_frame = 0.0;  // moving average
    double realtime_ratio = 1.0;   // sim time / wall time over the last measuring window
    bool behind = false;           // ratio clearly below 1 while running
};

// Stats panel (top left): scenario, timing, energies, contact and display state.
void draw_stats(const SimDriver& driver, const FrameMetrics& metrics, const UiState& ui,
                const RenderStats& stats);
// Color legend (top right) for the current color mode.
void draw_legend(const SimDriver& driver, const UiState& ui);
// Controls list (bottom left), or a one-line hint when hidden.
void draw_help(const UiState& ui);
// The current notice, if any (bottom centre).
void draw_message(const UiState& ui, double now);

}  // namespace hp::viewer
