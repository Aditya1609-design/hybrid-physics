// Live viewer. For now it opens a window and draws a test cube, to check that the graphics
// stack builds and runs on this platform.
// Usage: hp_view [--frames N] [--screenshot out.png]
//   --frames      close the window after N rendered frames (for smoke tests)
//   --screenshot  save the last rendered frame as a PNG when the window closes

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "raylib.h"

namespace {

// Positive integer argument, or -1 if the text is not one.
long parse_count(const char* text) {
    char* end = nullptr;
    const long n = std::strtol(text, &end, 10);
    return (end != text && *end == '\0' && n > 0) ? n : -1;
}

}  // namespace

int main(int argc, char** argv) {
    long max_frames = -1;
    std::string screenshot_path;
    for (int i = 1; i < argc; ++i) {
        const bool has_value = i + 1 < argc;
        if (std::strcmp(argv[i], "--frames") == 0 && has_value &&
            (max_frames = parse_count(argv[i + 1])) > 0) {
            ++i;
        } else if (std::strcmp(argv[i], "--screenshot") == 0 && has_value) {
            screenshot_path = argv[++i];
        } else {
            std::fprintf(stderr, "usage: %s [--frames N] [--screenshot out.png]\n", argv[0]);
            return 2;
        }
    }

    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT |
                   FLAG_WINDOW_HIGHDPI);
    InitWindow(1280, 800, "hp_view");

    Camera3D camera{};
    camera.position = {3.0f, 2.5f, 3.0f};
    camera.target = {0.0f, 0.5f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    long frame = 0;
    double fps_sum = 0.0;
    while (!WindowShouldClose() && (max_frames < 0 || frame < max_frames)) {
        BeginDrawing();
        ClearBackground({30, 32, 38, 255});
        BeginMode3D(camera);
        DrawCube({0.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, {90, 140, 200, 255});
        DrawCubeWires({0.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, RAYWHITE);
        DrawGrid(10, 0.5f);
        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
        ++frame;
        if (frame > 30) fps_sum += static_cast<double>(GetFPS());
    }

    // Average FPS after a short warm-up, so smoke runs can report it.
    if (frame > 30) {
        std::printf("frames: %ld, mean fps: %.1f\n", frame, fps_sum / static_cast<double>(frame - 30));
    }
    // Read back the last frame before the window goes away.
    if (!screenshot_path.empty()) {
        Image shot = LoadImageFromScreen();
        const bool ok = ExportImage(shot, screenshot_path.c_str());
        UnloadImage(shot);
        if (!ok) {
            std::fprintf(stderr, "error: cannot write %s\n", screenshot_path.c_str());
            CloseWindow();
            return 1;
        }
    }
    CloseWindow();
    return 0;
}
