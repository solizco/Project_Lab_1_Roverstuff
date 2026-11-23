# Archive

These are earlier development stages of the rover firmware and the bring-up programs written along the way. They are kept to show how the project developed. They are **not part of the build**, and they are not maintained against the current modules in `src/`. Most of them used `src/config_pins.h` and the driver files as they were at the time, so the pin numbers in their comments may be out of date.

| File | Stage |
|---|---|
| `Camera_test_main.cpp` | Phase 1: Arducam Mega bring-up (JPEG streaming and on-device red detection) |
| `visualizer_phase1.py` | PC viewer for `Camera_test_main.cpp` |
| `tof_test_main.cxx` | Phase 2: three VL53L1X sensors using the C API in `libs/VL53L1X-C-API-Pico` |
| `tof_test2.cpp` | Mixed VL53L0X/VL53L1X sensor test using the Pololu drivers |
| `tof_camera_fusion_main.cpp` | Phase 3: camera and ToF fusion with a drive state machine |
| `main.cpp`, `full_integration_main.cpp` | Early Phase 4 integration drafts, replaced by `src/rover_main.cpp`. They call a ball-handler function that was later removed, so they no longer compile. |
| `wheel_encoder_test.cpp` | Encoder odometry and navigation test |
| `buttonv2.cpp` | Debounced start/E-stop button driver. The button was never wired, so the build uses the stub in `src/button.cpp`. |
