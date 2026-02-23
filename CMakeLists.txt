cmake_minimum_required(VERSION 3.16)
project(haptic_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenHaptics SDK
set(OH_SDK "$ENV{OH_SDK_BASE}")
if(NOT OH_SDK)
    set(OH_SDK "C:/OpenHaptics/Developer/3.5.0")
endif()

if(NOT EXISTS "${OH_SDK}/include/HD/hd.h")
    message(FATAL_ERROR
        "OpenHaptics SDK not found at: ${OH_SDK}\n"
        "Set the OH_SDK_BASE environment variable or install to C:/OpenHaptics/Developer/3.5.0")
endif()

add_executable(haptic_server src/main.cpp)

target_include_directories(haptic_server PRIVATE
    "${OH_SDK}/include"
    "${OH_SDK}/utilities/include"
)

target_link_directories(haptic_server PRIVATE
    "${OH_SDK}/lib/x64/Release"
    "${OH_SDK}/utilities/lib/x64/Release"
)

target_link_libraries(haptic_server PRIVATE
    hd
    ws2_32
)
