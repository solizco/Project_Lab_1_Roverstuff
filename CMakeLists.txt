cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350)

# Locates the Pico SDK from PICO_SDK_PATH (environment or -D), or fetches it
# from GitHub when PICO_SDK_FETCH_FROM_GIT is set. Must come before project().
include(pico_sdk_import.cmake)

project(projectlab_rover C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ── Arducam Mega camera library ──────────────────────────────────────────────
add_subdirectory(libs/Arducam_Mega/src)

# ── Competition firmware ─────────────────────────────────────────────────────
add_executable(projectlab_rover
    src/rover_main.cpp
    src/button.cpp
    src/collector_arms.cpp
    src/color_sensor.cpp
    src/rack_pinion.cpp
    src/flywheel_motors.cpp
    src/rover_motors.cpp
    src/ball_handler.cpp
    src/wheel_encoder.cpp
    src/VL53L0X.cpp
    src/VL53L1X.cpp
)

target_include_directories(projectlab_rover PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src/Arducam
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src/pico_hal
)

target_link_libraries(projectlab_rover
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_pwm
    arducam_mega
)

pico_enable_stdio_usb(projectlab_rover 1)
pico_enable_stdio_uart(projectlab_rover 0)
pico_add_extra_outputs(projectlab_rover)

# ── Camera blob-detection test ───────────────────────────────────────────────
# Stand-alone camera test: no ToF, motors or ball handler. Streams RGB565
# frames and detection results to tools/vision_visualizer.py over USB.
add_executable(phase1b_blob
    src/camera_blob_test.cpp
)

target_include_directories(phase1b_blob PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src/Arducam
    ${CMAKE_CURRENT_LIST_DIR}/libs/Arducam_Mega/src/pico_hal
)

target_link_libraries(phase1b_blob
    pico_stdlib
    hardware_spi
    hardware_gpio
    arducam_mega
)

pico_enable_stdio_usb(phase1b_blob 1)
pico_enable_stdio_uart(phase1b_blob 0)
pico_add_extra_outputs(phase1b_blob)
