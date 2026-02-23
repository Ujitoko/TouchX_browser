# TouchX Browser

Geomagic TouchX haptic device to browser bridge.
A C++ WebSocket server streams real-time position, force and button state from the TouchX to a Three.js 3D visualization in the browser.

## Architecture

```
TouchX device
    |
    v
OpenHaptics HD API (1 kHz servo loop)
    |  mutex
    v
Main loop (~60 Hz) ─── WebSocket ──> Browser (Three.js)
                   ─── HTTP ──────> web/index.html
                        port 8080
```

- **Servo loop (1 kHz)**: Reads stylus position, computes sphere collision force feedback via OpenHaptics HD API.
- **Main loop (~60 Hz)**: Sends JSON state (`pos`, `force`, `btn`, `touch`, `model`) over WebSocket.
- **Browser**: Three.js scene with a translucent sphere, stylus cursor, force arrow, and OrbitControls.

## Prerequisites

- Windows 10/11 (x64)
- Visual Studio 2022
- [OpenHaptics SDK 3.5+](https://support.3dsystems.com/s/article/OpenHaptics-for-Windows-Developer-Edition-v35) installed at `C:\OpenHaptics\Developer\3.5.0` (or set `OH_SDK_BASE` env var)
- Geomagic TouchX device connected and drivers installed

## Build

```bat
build.bat
```

Or manually:

```bat
cmake -B build -S . -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

## Run

```bat
build\Release\haptic_server.exe
```

Open http://localhost:8080 in a browser.

## JSON Protocol

The server sends text WebSocket frames at ~60 Hz:

```json
{
  "pos": [x, y, z],
  "btn": 0,
  "touch": false,
  "force": [fx, fy, fz],
  "model": "Touch X"
}
```

| Field   | Description                                    |
|---------|------------------------------------------------|
| `pos`   | Stylus tip position in mm (device coordinates) |
| `btn`   | Button bitmask                                 |
| `touch` | `true` if stylus is inside the sphere          |
| `force` | Force feedback vector in N                     |
| `model` | Device model string                            |

## Project Structure

```
CMakeLists.txt      CMake build configuration
build.bat           One-click build script (VS2022)
src/main.cpp        C++ server (haptic + HTTP + WebSocket)
web/index.html      Three.js frontend
```

## License

MIT
