#ifndef CAMERA_H
#define CAMERA_H

#include <cstddef>
#include <cstdint>
#include <string>

// A borrowed view of the most recent captured frame.
//
// Deliberately free of driver types: this header is included by every board,
// and the two camera implementations disagree about which driver they use
// (esp32-camera's camera_fb_t, esp_video's V4L2 buffers). `fourcc` carries the
// V4L2 pixel format so a consumer that cares can compare it against
// V4L2_PIX_FMT_* after including the header that defines those — nothing is
// assumed about the layout here.
//
// `data` points into the driver's own buffer and is valid only until the next
// capture. Copy it if you need to keep it.
struct CameraFrame {
    const uint8_t* data = nullptr;
    size_t len = 0;
    int width = 0;
    int height = 0;
    uint32_t fourcc = 0;   // 0 = unknown
};

class Camera {
public:
    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op

    // Optional: lend the most recent frame to something running on the device.
    //
    // Default false, so every existing board keeps compiling and keeps
    // behaving exactly as before — a board that cannot or should not share
    // pixels simply does not override this. Added for on-device face
    // detection, which otherwise has no way to see what the camera sees:
    // Capture() and Explain() between them only ever send the image away.
    virtual bool PeekFrame(CameraFrame* out) { return false; }
    virtual std::string Explain(const std::string& question) = 0;
};

#endif // CAMERA_H
