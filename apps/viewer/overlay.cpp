#include "overlay.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <set>

#include "raylib.h"

namespace hp::viewer {

namespace {

constexpr int kFont = 18;
constexpr int kLine = 22;
constexpr int kPad = 10;
const Color kText{225, 228, 235, 255};
const Color kDim{150, 156, 168, 255};
const Color kWarn{240, 196, 90, 255};
const Color kBad{240, 110, 100, 255};
const Color kPanel{16, 18, 22, 190};

// Lines of one panel, collected first so the background can be sized to fit. A tab in a
// line starts a second column, aligned across the panel (the font is proportional, so
// spaces cannot line text up).
class Panel {
public:
    void add(Color color, const char* fmt, ...) {
        char buf[256];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        lines_.push_back({buf, color});
    }
    void gap() { lines_.push_back({"", kText}); }

    // Draws at (x, y); x < 0 means right-aligned that far from the right edge.
    void draw(int x, int y) const {
        int col = 0;  // x offset of the second column
        for (const Line& l : lines_) {
            const size_t tab = l.text.find('\t');
            if (tab != std::string::npos) {
                col = std::max(col, MeasureText(l.text.substr(0, tab).c_str(), kFont) + 2 * kPad);
            }
        }
        int width = 0;
        for (const Line& l : lines_) {
            const size_t tab = l.text.find('\t');
            const int w = tab == std::string::npos
                              ? MeasureText(l.text.c_str(), kFont)
                              : col + MeasureText(l.text.substr(tab + 1).c_str(), kFont);
            width = std::max(width, w);
        }
        const int w = width + 2 * kPad;
        const int h = static_cast<int>(lines_.size()) * kLine + 2 * kPad - (kLine - kFont);
        if (x < 0) x = GetScreenWidth() + x - w;
        DrawRectangle(x, y, w, h, kPanel);
        int ty = y + kPad;
        for (const Line& l : lines_) {
            const size_t tab = l.text.find('\t');
            if (tab == std::string::npos) {
                DrawText(l.text.c_str(), x + kPad, ty, kFont, l.color);
            } else {
                DrawText(l.text.substr(0, tab).c_str(), x + kPad, ty, kFont, l.color);
                DrawText(l.text.substr(tab + 1).c_str(), x + kPad + col, ty, kFont, l.color);
            }
            ty += kLine;
        }
    }

private:
    struct Line {
        std::string text;
        Color color;
    };
    std::vector<Line> lines_;
};

Color to_color(Rgb c) { return {c.r, c.g, c.b, 255}; }

}  // namespace

void draw_stats(const SimDriver& driver, const FrameMetrics& m, const UiState& ui,
                const RenderStats& stats) {
    const Scenario& sc = driver.scenario();
    const Scene& scene = driver.scene();
    Panel p;
    p.add(kText, "%s", sc.name.c_str());
    p.add(kDim, "%d vertices, %d tets, %zu colliders", scene.state.num_vertices(),
          scene.mesh.num_tets(), scene.solver.colliders().size());
    p.add(kDim, "%s preset: %d substeps x %d iterations", sc.preset.c_str(),
          sc.solver.substeps, sc.solver.iterations);
    p.gap();

    const char* run_state = driver.failed() ? "STOPPED: state became non-finite"
                            : ui.paused     ? "paused"
                                            : "running";
    p.add(driver.failed() ? kBad : kText, "frame %d / %d   t = %.2f s   %s", driver.frame(),
          sc.frames, driver.sim_time(), run_state);
    p.add(kText, "render %d fps   step %.2f ms (last %.2f)", stats.fps, driver.mean_step_ms(),
          driver.last_step_ms());
    p.add(kDim, "sim speed %.2fx real time, %.2f steps per render frame",
          stats.realtime_ratio, stats.steps_per_frame);
    if (stats.behind) p.add(kWarn, "slower than real time: the solver cannot keep up");
    p.gap();

    p.add(kText, "kinetic\t%.4g J", m.energy.kinetic);
    p.add(kText, "gravitational\t%.4g J", m.energy.gravitational);
    p.add(kText, "elastic\t%.4g J", m.energy.elastic);
    p.add(kText, "total\t%.4g J", m.energy.total());
    if (sc.preset == "realtime") {
        // Known limitation since Phase 4: the real-time preset is far from converged.
        p.add(kDim, "real-time preset: elastic is unconverged and");
        p.add(kDim, "spikes on impact; not physical energy");
    }
    p.gap();

    p.add(kText, "max penetration %.3g m, %d vertices in contact", m.penetration.max_depth,
          m.penetration.num_in_contact);
    p.add(kText, "edge error rms %.3g, volume error rms %.3g", m.error.edge_rms,
          m.error.volume_rms);
    if (m.error.inverted_tets > 0) p.add(kWarn, "%d inverted tets", m.error.inverted_tets);
    p.gap();

    p.add(kDim, "display: %s%s   follow: %s", display_mode_name(ui.display),
          ui.tet_edges ? " + tet edges" : "", ui.follow ? "on" : "off");
    for (const std::string& w : scene.warnings) p.add(kWarn, "warning: %s", w.c_str());
    p.draw(kPad, kPad);
}

void draw_legend(const SimDriver& driver, const UiState& ui) {
    Panel p;
    const SimState& s = driver.scene().state;
    p.add(kText, "color: %s%s", color_mode_name(ui.color_mode), ui.preview ? "  (PREVIEW)" : "");
    if (ui.preview && (ui.color_mode == ColorMode::Owner || ui.color_mode == ColorMode::Region)) {
        p.add(kWarn, "display-only pattern; the simulation");
        p.add(kWarn, "state is unchanged (P to turn off)");
    }
    switch (ui.color_mode) {
        case ColorMode::Owner: {
            p.add(to_color(owner_color(SolverId::Xpbd)), "XPBD");
            p.add(to_color(owner_color(SolverId::Gnn)), "GNN");
            if (!ui.preview) {
                size_t gnn = 0;
                for (SolverId o : s.solver_owner) gnn += o == SolverId::Gnn ? 1 : 0;
                p.add(kDim, "%zu of %zu vertices GNN-owned", gnn, s.solver_owner.size());
            }
            break;
        }
        case ColorMode::Region: {
            if (!ui.preview) {
                const std::set<Index> ids(s.region_id.begin(), s.region_id.end());
                p.add(kDim, "%zu region(s) in the state", ids.size());
            }
            p.add(kDim, "palette repeats every 12 ids");
            break;
        }
        case ColorMode::Body: {
            p.add(kDim, "%zu body(ies)", s.bodies.size());
            break;
        }
        case ColorMode::Plain: break;
    }
    p.draw(-kPad, kPad);
}

void draw_help(const UiState& ui) {
    if (!ui.show_help) {
        DrawText("H: controls", kPad, GetScreenHeight() - kPad - kFont, kFont, kDim);
        return;
    }
    Panel p;
    p.add(kText, "left drag\torbit");
    p.add(kText, "right drag\tpan (also middle drag, or shift + left drag)");
    p.add(kText, "wheel\tzoom");
    p.add(kText, "F / V\tframe the scene / follow the body");
    p.add(kText, "space / .\tpause or run / one frame while paused");
    p.add(kText, "R / L\treset to frame 0 / reload the file from disk");
    p.add(kText, "W\tsolid / solid + wire / wire");
    p.add(kText, "T\tall tet edges");
    p.add(kText, "C\tcolor: owner / region / body / plain");
    p.add(kText, "P\towner/region preview pattern (display only)");
    p.add(kText, "Tab / F12\tstats panel / screenshot");
    p.add(kText, "H / Esc\thide controls / quit");
    const int h = 12 * kLine + 2 * kPad - (kLine - kFont);
    p.draw(kPad, GetScreenHeight() - kPad - h);
}

void draw_message(const UiState& ui, double now) {
    if (ui.message.empty() || now > ui.message_until) return;
    const int w = MeasureText(ui.message.c_str(), kFont);
    const int x = (GetScreenWidth() - w) / 2;
    const int y = GetScreenHeight() - 3 * kLine;
    DrawRectangle(x - kPad, y - kPad / 2, w + 2 * kPad, kFont + kPad, kPanel);
    DrawText(ui.message.c_str(), x, y, kFont, kWarn);
}

}  // namespace hp::viewer
