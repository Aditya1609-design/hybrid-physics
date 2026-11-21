// Live viewer: runs a scenario file in a window in real time.
// Usage: hp_view <scenario.json> [--preset realtime|reference] [--frames N]
//                [--screenshot out.png]
//   --preset      replace the scenario's substep and iteration counts with a preset's
//   --frames      close after N simulation frames and print the final state hash, which
//                 matches `hp_run --frames N` for the same scenario and preset
//   --screenshot  with --frames, save the frame showing frame N as a PNG (F12 saves one
//                 to the working directory at any time)
// Exit codes: 0 ok, 1 invalid scenario or unwritable screenshot, 2 usage error.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/colors.hpp"
#include "core/orbit_camera.hpp"
#include "core/sim_driver.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/solver/xpbd.hpp"
#include "raylib.h"
#include "render.hpp"
#include "rlgl.h"

namespace {

using hp::Vec3;
using namespace hp::viewer;

struct Options {
    std::string scenario_path;
    std::string preset;
    long frames = -1;  // -1: run until the window is closed
    std::string screenshot_path;
};

int usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s <scenario.json> [--preset realtime|reference] [--frames N]\n"
                 "           [--screenshot out.png (needs --frames)]\n",
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
        } else if (std::strcmp(a, "--screenshot") == 0 && has_value) {
            o.screenshot_path = argv[++i];
        } else if (a[0] != '-' && o.scenario_path.empty()) {
            o.scenario_path = a;
        } else {
            return false;
        }
    }
    // A screenshot is taken on the last frame of a --frames run.
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
void handle_camera_input(OrbitCamera& cam) {
    const Vector2 d = GetMouseDelta();
    const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    const bool left = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    if ((left && shift) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT) ||
        IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        cam.pan(d.x, d.y, static_cast<float>(GetScreenHeight()));
    } else if (left) {
        cam.orbit(-0.008f * d.x, 0.008f * d.y);
    }
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) cam.zoom(std::pow(0.88f, wheel));
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

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return usage(argv[0]);

    std::optional<SimDriver> driver;
    try {
        driver.emplace(load(opt));
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
    const Vector2 dpi = GetWindowScaleDPI();
    std::printf("scenario:  %s, %d vertices, %s preset\n", driver->scenario().name.c_str(),
                driver->scene().state.num_vertices(), driver->scenario().preset.c_str());
    std::printf("window:    %dx%d points, framebuffer %dx%d, display scale %.2f\n",
                GetScreenWidth(), GetScreenHeight(), GetRenderWidth(), GetRenderHeight(),
                static_cast<double>(dpi.x));

    OrbitCamera cam;
    frame_camera(cam, driver->scene());
    DisplayMode display = DisplayMode::Solid;
    bool tet_edges = false;

    // GPU resources live inside this scope so they are released before CloseWindow.
    long render_frames = 0;
    double fps_sum = 0.0;
    {
        auto body = std::make_unique<BodyRenderer>(driver->scene().topology);
        ColliderRenderer colliders;
        std::vector<Rgb> colors;

        bool screenshot_ok = true;
        while (!WindowShouldClose()) {
            handle_camera_input(cam);
            if (IsKeyPressed(KEY_W)) display = next_display_mode(display);
            if (IsKeyPressed(KEY_T)) tet_edges = !tet_edges;
            if (IsKeyPressed(KEY_F)) frame_camera(cam, driver->scene());

            // With --frames, stepping stops exactly on frame N so the hash matches hp_run.
            const hp::Index limit = opt.frames > 0 ? static_cast<hp::Index>(opt.frames)
                                                   : std::numeric_limits<hp::Index>::max();
            driver->advance(static_cast<double>(GetFrameTime()), limit);

            const hp::SimState& s = driver->scene().state;
            vertex_colors(ColorMode::Owner, s.region_id, s.solver_owner, s.body_id, colors);
            body->update(s, colors, cam);

            BeginDrawing();
            ClearBackground({30, 32, 38, 255});
            BeginMode3D(to_raylib(cam));
            body->draw(display, tet_edges, cam);
            colliders.draw(driver->scene().solver.colliders(), cam);
            EndMode3D();
            DrawFPS(10, 10);

            // The back buffer is undefined after the swap, so read it before EndDrawing.
            const bool last = opt.frames > 0 && driver->frame() >= opt.frames;
            if (last && !opt.screenshot_path.empty()) {
                rlDrawRenderBatchActive();
                screenshot_ok = save_screenshot(opt.screenshot_path);
            }
            EndDrawing();

            ++render_frames;
            if (render_frames > 30) fps_sum += static_cast<double>(GetFPS());
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
