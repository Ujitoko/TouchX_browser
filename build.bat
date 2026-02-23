@echo off
setlocal

echo === TouchX Haptic Demo Build ===

:: Find Visual Studio installation via vswhere
set "VS_PATH="
for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath 2^>nul`) do (
    set "VS_PATH=%%i"
)

if not defined VS_PATH (
    echo ERROR: Visual Studio not found.
    exit /b 1
)

echo Found VS at: %VS_PATH%
call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1

cmake -B build -S . -G "Visual Studio 17 2022" -A x64
if errorlevel 1 (
    echo CMake configure failed.
    exit /b 1
)

cmake --build build --config Release
if errorlevel 1 (
    echo Build failed!
    exit /b 1
)

echo.
echo === Build successful ===
echo Run: build\Release\haptic_server.exe
