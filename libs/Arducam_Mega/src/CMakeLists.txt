# CMakeLists.txt for Arducam Mega library (Pico platform)
# Place this file at: libs/Arducam_Mega/src/CMakeLists.txt
#
# This pulls in the core SDK files from src/Arducam/ and the Pico SPI
# driver from the full_featured example.

project(arducam_mega C CXX)

set(DIR_CAM_SRCS
    # Core SDK (camera protocol + API)
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam/ArducamCamera.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam/ArducamCamera.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam/picoHal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam/Platform.h

    # C++ wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam_Mega.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam_Mega.h

    # Pico SPI HAL driver (copied from examples/Pico/full_featured/Arducam/)
    ${CMAKE_CURRENT_SOURCE_DIR}/pico_hal/cam_spi_master.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pico_hal/cam_spi_master.h
)

add_library(arducam_mega ${DIR_CAM_SRCS})

target_include_directories(arducam_mega PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Arducam
    ${CMAKE_CURRENT_SOURCE_DIR}/pico_hal
)

target_link_libraries(arducam_mega PUBLIC
    hardware_spi
    pico_stdlib
)