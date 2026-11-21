// Live viewer: runs a scenario file in a window in real time.
// Usage: hp_view <scenario.json> [--preset realtime|reference] [--paused] [--frames N]
//                [--screenshot out.png]
//   --preset      replace the scenario's substep and iteration counts with a preset's
//   --paused      open paused at frame 0 (space runs, '.' steps one frame)
//   --frames      close after N simulation frames and print the final state hash, which
//                 matches `hp_run --frames N` for the same scenario and preset
//   --screenshot  with --frames, save the frame showing frame N as a PNG (F12 saves one
//                 to the working directory at any time)
// Press H in the window for the controls.
// Exit codes: 0 ok, 1 invalid scenario or unwritable screenshot, 2 usage error.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/colors.hpp"
#include "core/orbit_camera.hpp"
#include "core/sim_driver.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/solver/xpbd.hpp"
#include "overlay.hpp"
#include "raylib.h"
#include "render.hpp"
#include "rlgl.h"

namespace {

using hp::Vec3;
using namespace hp::viewer;

// Slabs in the owner/region preview pattern.
constexpr hp::Index kPreviewSlabs = 4;

struct Options {
    std::string scenario_path;
    std::string preset;
    bool paused = false;
    long frames = -1;  // -1: run until the window is closed
    std::string screenshot_path;
};

int usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s <scenario.json> [--preset realtime|reference] [--paused]\n"
                 "           [--frames N] [--screenshot out.png (needs --frames)]\n",
                 argv0);
    return 2;
}

// Positive integer argument, or -1 if the text is not one.
long parse_count(const char* text) {
    char* end = nullptr;
    const long n = std::strtol(text, &end, 10);
    return (end != text && *end == '\0' && n > 0) ? n : -1;
}

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool has_value = i + 1 < argc;
        if (std::strcmp(a, "--frames") == 0 && has_value) {
            if ((o.frames = parse_count(argv[++i])) < 0) return false;
        } else if (std::strcmp(a, "--preset") == 0 && has_value) {
            o.preset = argv[++i];
            if (o.preset != "realtime" && o.preset != "reference") return false;
        } else if (std::strcmp(a, "--paused") == 0) {
            o.paused = true;
        } else if (std::strcmp(a, "--screenshot") == 0 && has_value) {
            o.screenshot_path = argv[++i];
        } else if (a[0] != '-' && o.scenario_path.empty()) {
            o.scenario_path = a;
        } else {
            return false;
        }
    }
    // A screenshot is taken on the last frame of a --frames run, and a paused run with
    // --frames would never get there.
    if (o.frames > 0 && o.paused) return false;
    return !o.scenario_path.empty() && (o.screenshot_path.empty() || o.frames > 0);
}

// Loads the scenario and applies the preset override the same way hp_run does.
hp::Scenario load(const Options& opt) {
    hp::Scenario s = hp::load_scenario(opt.scenario_path);
    if (!opt.preset.empty()) {
        const hp::XpbdSettings p = opt.preset == "reference" ? hp::XpbdSettings::reference()
                                                             : hp::XpbdSettings::realtime();
        s.solver.substeps = p.substeps;
        s.solver.iterations = p.iterations;
        s.preset = opt.preset;
    }
    return s;
}

// Bounds of the body plus any sphere and box colliders (planes are unbounded).
void scene_bounds(const hp::Scene& scene, Vec3& lo, Vec3& hi) {
    lo = Vec3::Constant(std::numeric_limits<float>::max());
    hi = Vec3::Constant(std::numeric_limits<float>::lowest());
    for (const Vec3& x : scene.state.positions) {
        lo = lo.cwiseMin(x);
        hi = hi.cwiseMax(x);
    }
    for (const hp::Collider& c : scene.solver.colliders()) {
        Vec3 r = Vec3::Zero();
        if (c.kind == hp::ColliderKind::Sphere) r = Vec3::Constant(c.radius);
        if (c.kind == hp::ColliderKind::Box) r = c.rotation.cwiseAbs() * c.half_extents;
        if (c.kind == hp::ColliderKind::Plane) continue;
        lo = lo.cwiseMin(c.center - r);
        hi = hi.cwiseMax(c.center + r);
    }
}

// Frames the scene, padded to at least 1.5 m across so a small body keeps some context.
void frame_camera(OrbitCamera& cam, const hp::Scene& scene) {
    Vec3 lo;
    Vec3 hi;
    scene_bounds(scene, lo, hi);
    const Vec3 centre = 0.5f * (lo + hi);
    const Vec3 half = (0.5f * (hi - lo)).cwiseMax(Vec3::Constant(0.75f));
    lo = centre - half;
    hi = centre + half;
    const float aspect =
        static_cast<float>(GetScreenWidth()) / static_cast<float>(std::max(GetScreenHeight(), 1));
    cam.frame(lo, hi, aspect);
}

Vec3 centroid(const hp::SimState& s) {
    Vec3 sum = Vec3::Zero();
    for (const Vec3& x : s.positions) sum += x;
    return s.positions.empty() ? sum : Vec3(sum / static_cast<float>(s.positions.size()));
}

Camera3D to_raylib(const OrbitCamera& cam) {
    Camera3D c{};
    const Vec3 p = cam.position();
    const Vec3 u = cam.up();
    c.position = {p.x(), p.y(), p.z()};
    c.target = {cam.target.x(), cam.target.y(), cam.target.z()};
    c.up = {u.x(), u.y(), u.z()};
    c.fovy = cam.fovy_deg;
    c.projection = CAMERA_PERSPECTIVE;
    return c;
}

// Mouse: left drag orbits, right or middle drag (or shift + left) pans, wheel zooms.
// Returns true if the user panned this frame.
bool handle_camera_input(OrbitCamera& cam) {
    const Vector2 d = GetMouseDelta();
    const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    const bool left = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    bool panned = false;
    if ((left && shift) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT) ||
        IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        cam.pan(d.x, d.y, static_cast<float>(GetScreenHeight()));
        panned = d.x != 0.0f || d.y != 0.0f;
    } else if (left) {
        cam.orbit(-0.008f * d.x, 0.008f * d.y);
    }
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) cam.zoom(std::pow(0.88f, wheel));
    return panned;
}

// Reads the whole framebuffer (window size times the display scale) and writes a PNG.
bool save_screenshot(const std::string& path) {
    Image shot{};
    shot.width = GetRenderWidth();
    shot.height = GetRenderHeight();
    shot.mipmaps = 1;
    shot.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    shot.data = rlReadScreenPixels(shot.width, shot.height);
    const bool ok = ExportImage(shot, path.c_str());
    UnloadImage(shot);
    return ok;
}

void notify(UiState& ui, const std::string& text) {
    ui.message = text;
    ui.message_until = GetTime() + 4.0;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return usage(argv[0]);

    std::unique_ptr<SimDriver> driver;
    try {
        driver = std::make_unique<SimDriver>(load(opt));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    for (const std::string& w : driver->scene().warnings) {
        std::fprintf(stderr, "warning: %s\n", w.c_str());
    }

    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT |
                   FLAG_WINDOW_HIGHDPI);
    InitWindow(1280, 800, ("hp_view - " + driver->scenario().name).c_str());
    SetExitKey(KEY_ESCAPE);
    const Vector2 dpi = GetWindowScaleDPI();
    std::printf("scenario:  %s, %d vertices, %s preset\n", driver->scenario().name.c_str(),
                driver->scene().state.num_vertices(), driver->scenario().preset.c_str());
    std::printf("window:    %dx%d points, framebuffer %dx%d, display scale %.2f\n",
                GetScreenWidth(), GetScreenHeight(), GetRenderWidth(), GetRenderHeight(),
                static_cast<double>(dpi.x));

    OrbitCamera cam;
    frame_camera(cam, driver->scene());
    UiState ui;
    ui.paused = opt.paused;
    RenderStats stats;
    // Sim time and wall time over the current measuring window, for the real-time ratio.
    double window_wall = 0.0;
    double window_sim = 0.0;

    // With --frames, stepping stops exactly on frame N so the hash matches hp_run.
    const hp::Index limit = opt.frames > 0 ? static_cast<hp::Index>(opt.frames)
                                           : std::numeric_limits<hp::Index>::max();

    // GPU resources live inside this scope so they are released before CloseWindow.
    long render_frames = 0;
    double fps_sum = 0.0;
    {
        auto body = std::make_unique<BodyRenderer>(driver->scene().topology);
        ColliderRenderer colliders;
        OwnerPreview preview = make_owner_preview(driver->scene().state.rest_positions,
                                                  kPreviewSlabs);
        std::vector<Rgb> colors;
        bool screenshot_ok = true;

        while (!WindowShouldClose()) {
            const double now = GetTime();

            // Camera and display toggles.
            if (handle_camera_input(cam)) ui.follow = false;
            if (IsKeyPressed(KEY_F)) frame_camera(cam, driver->scene());
            if (IsKeyPressed(KEY_V)) ui.follow = !ui.follow;
            if (IsKeyPressed(KEY_W)) ui.display = next_display_mode(ui.display);
            if (IsKeyPressed(KEY_T)) ui.tet_edges = !ui.tet_edges;
            if (IsKeyPressed(KEY_C)) ui.color_mode = next_color_mode(ui.color_mode);
            if (IsKeyPressed(KEY_P)) {
                ui.preview = !ui.preview;
                // The pattern only shows in owner and region modes.
                if (ui.preview && ui.color_mode != ColorMode::Region) {
                    ui.color_mode = ColorMode::Owner;
                }
            }
            if (IsKeyPressed(KEY_TAB)) ui.show_stats = !ui.show_stats;
            if (IsKeyPressed(KEY_H)) ui.show_help = !ui.show_help;

            // Simulation controls.
            if (IsKeyPressed(KEY_SPACE)) ui.paused = !ui.paused;
            if (IsKeyPressed(KEY_R)) {
                driver->reset();
                notify(ui, "reset to frame 0");
            }
            if (IsKeyPressed(KEY_L)) {
                // The new scene is built completely before it replaces the running one, so
                // an edited file that does not load leaves the old scene untouched.
                try {
                    driver = std::make_unique<SimDriver>(load(opt));
                    body = std::make_unique<BodyRenderer>(driver->scene().topology);
                    preview = make_owner_preview(driver->scene().state.rest_positions,
                                                 kPreviewSlabs);
                    frame_camera(cam, driver->scene());
                    notify(ui, "reloaded " + opt.scenario_path);
                    for (const std::string& w : driver->scene().warnings) {
                        std::fprintf(stderr, "warning: %s\n", w.c_str());
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "error: %s\n", e.what());
                    notify(ui, std::string("reload failed: ") + e.what());
                }
            }

            int steps = 0;
            if (ui.paused) {
                if (IsKeyPressed(KEY_PERIOD) && driver->frame() < limit) {
                    driver->step();
                    steps = 1;
                }
            } else {
                steps = driver->advance(static_cast<double>(GetFrameTime()), limit);
            }
            stats.steps_per_frame = 0.95 * stats.steps_per_frame + 0.05 * steps;
            stats.fps = GetFPS();

            // Real-time ratio over half-second windows. The first frames include window
            // start-up time, so measuring starts a few frames in; pausing restarts it.
            if (ui.paused || driver->failed() || render_frames < 5) {
                window_wall = 0.0;
                window_sim = 0.0;
            } else {
                window_wall += static_cast<double>(GetFrameTime());
                window_sim += steps * static_cast<double>(hp::kFrameDt);
                if (window_wall >= 0.5) {
                    stats.realtime_ratio = window_sim / window_wall;
                    window_wall = 0.0;
                    window_sim = 0.0;
                }
            }
            stats.behind = !ui.paused && stats.realtime_ratio < 0.9;

            const hp::SimState& s = driver->scene().state;
            if (ui.follow) {
                // Ease towards the body so the view does not jerk.
                const float k = std::min(1.0f, 6.0f * GetFrameTime());
                cam.target += k * (centroid(s) - cam.target);
            }
            if (ui.preview) {
                vertex_colors(ui.color_mode, preview.region_id, preview.solver_owner, s.body_id,
                              colors);
            } else {
                vertex_colors(ui.color_mode, s.region_id, s.solver_owner, s.body_id, colors);
            }
            body->update(s, colors, cam);

            BeginDrawing();
            ClearBackground({30, 32, 38, 255});
            BeginMode3D(to_raylib(cam));
            body->draw(ui.display, ui.tet_edges, cam);
            colliders.draw(driver->scene().solver.colliders(), cam);
            EndMode3D();
            if (ui.show_stats) {
                draw_stats(*driver, driver->metrics(), ui, stats);
                draw_legend(*driver, ui);
            }
            draw_help(ui);
            draw_message(ui, now);

            // The back buffer is undefined after the swap, so read it before EndDrawing.
            const bool last = opt.frames > 0 && driver->frame() >= opt.frames;
            if (last && !opt.screenshot_path.empty()) {
                rlDrawRenderBatchActive();
                screenshot_ok = save_screenshot(opt.screenshot_path);
            }
            EndDrawing();

            ++render_frames;
            if (render_frames > 30) fps_sum += static_cast<double>(stats.fps);
            if (last) break;
        }
        if (!screenshot_ok) {
            std::fprintf(stderr, "error: cannot write %s\n", opt.screenshot_path.c_str());
            body.reset();
            CloseWindow();
            return 1;
        }
    }
    CloseWindow();

    std::printf("frames %d, sim time %.3f s", driver->frame(), driver->sim_time());
    if (render_frames > 30) {
        std::printf(", mean render fps %.1f",
                    fps_sum / static_cast<double>(render_frames - 30));
    }
    std::printf("\nfinal state hash %016" PRIx64 "\n", driver->state_hash());
    return 0;
}
