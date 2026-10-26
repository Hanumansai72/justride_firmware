# Motion, stationary speed and redraw correction — 2026-10-19

## Findings

1. The 2D HUD change disconnected the sensor from rendering: `map_frame_set_tilt()` did nothing, and the fallback pointer always received STRAIGHT. The MPU6500 continued reporting LEFT correctly, but no drawing code consumed that state.
2. On the phone, a valid GPS speed of zero was replaced by speed inferred from changing coordinates. Position drift could therefore become a nonzero speed. The initial ride screen also bypassed the canonical speed filter. The supplied BLE log itself consistently reports `Spd=0km/h`; it does not demonstrate a nonzero firmware speed value.
3. Identical marker packets were marked dirty. Marker painting incremented the same serial used for full-map changes, unnecessarily repainting trip/compass/notification overlays. POI updates and progress-only route packets also dirtied map drawing. Speed and maneuver cards were cleared before drawing their contents, making partial updates visibly blank between drawing steps.
4. The previous APK accepted only226×177 map JPEGs. A backend still returning198×194 caused a client-side format rejection with the generic message shown in the screenshot. This is a verified client incompatibility; the exact production response was not inspected. Other network/provider failures can still occur and now have more specific diagnostics.

## Changes

- The render task reads the existing calibrated sensor state and updates the rider pointer without a map download or manual screen refresh. Accelerometer lean and existing forward/backward classification drive its visual motion; gyro mode uses the existing relative yaw reference. This changes only the visual rider pointer. The route instruction remains the phone-provided TURN RIGHT in the supplied example, and GPS heading/compass and speed are not overwritten by tilt.
- Map pointer motion restores/draws only its21×21 background patch, including while a replacement map is being received. A2-degree drawing step limits unnecessary writes. The fallback route pointer also updates independently of route packets.
- Identical marker values produce no new marker redraw. Full-map and marker damage are tracked separately; overlays repaint for marker damage only when their own rectangle overlaps it. Progress-only and POI updates no longer force map repainting.
- New map rows replace the previous image directly without first clearing the map viewport.
- Speed and maneuver cards are composed in a reusable8-row RGB565 strip before sending finished pixels. This uses **5,120bytes**, shared between those regions, with no full-screen framebuffer or per-update allocation. The speed card uses11 strips; the maneuver banner uses8. Other SPI tasks cannot draw into that temporary buffer: it is owned by the drawing task under the existing recursive SPI lock.
- Speed is now taken only from the phone's reported GPS speed. Coordinate-derived speed fallback was removed. A valid zero is respected immediately; uncertainty including zero suppresses apparent movement; two usable positive samples are needed to start the meter; invalid/poor speed input is not converted into inferred motion. Existing UI behavior maps unavailable filtered speed to0. The initial ride state uses the same filtered source.
- The app accepts only the two known JPEG sizes, centers/crops legacy maps at their original scale, and copies their complete attribution strip. Unsupported dimensions, timeout and decode/network failures have distinguishable messages. No backend change is required solely to accept the legacy size.
- Fixed incorrect relative imports in the existing shared bottom-navigation and battery widgets, exposed by the fresh Flutter decoder verification. No widget appearance or behavior changed.

## Verification

- ESP-IDF5.5.5 build and size check: **PASS**, no final compiler warnings/errors.
- Static DRAM: **38,588bytes**, linker-reported remaining **85,992bytes**. IRAM: **117,027bytes**, remaining **14,045bytes**. Image size: **729,529bytes**. These are linker measurements, not runtime heap readings.
- Native C harness executed the production map state machine and TFT drawing primitives with hardware calls replaced by a host sink: **PASS**. LEFT and RIGHT samples changed pixels only within the rider marker's21×21 rectangle; full-map serial stayed unchanged; repeated identical input caused **zero SPI data writes**. A speed-card update left all pixels outside its88×82 rectangle unchanged. Host motion captures are saved under `build/capture/motion-left-3x.png` and `motion-right-3x.png`; they use synthetic background/input, not a real ride or physical screen capture.
- Dart speed checks: **PASS** for valid zero, uncertainty covering zero, isolated positive sample, sustained motion, stopping and missing/poor speed input.
- Actual Flutter JPEG decode/composition: **PASS** for198×194 and226×177 inputs,226×177 output, retained white attribution pixels and legacy side padding. The first fresh verification exposed two bad shared-widget imports; the corrected run passed.
- Analysis of changed speed/location/navigation/map services: **No issues found**.
- All temporary verification source files and image fixtures were removed. No new test files are retained in the codebase.

Physical I2C-to-LCD latency, stationary GPS behavior on the user's phone, actual display flicker, runtime heap, touch and BLE behavior still require installing/flashing this build. The render task targets20Hz (50ms cadence); no measured physical latency or actual20FPS claim is made. Existing telemetry records runtime render/heap/stack measurements for that check.

## Files for this follow-up

Firmware: `main/display/display.c`, `map_frame.c/.h`, `display_tft.c/.h`, `hud_ui.c`; host capture preparation utility updated for the strip renderer. Sensor acquisition, wiring, calibration, OTA, BLE packet format and route instructions were preserved.

App: speed filter, location/navigation services, device map compositor, shared-widget import paths, bundled firmware asset. No backend business logic changes.

This follow-up supersedes the original2D report's statements that IMU visualization was disabled and that the wider APK required a backend rollout to decode older maps. Provider credentials/authentication still need to be configured correctly for live map requests.

Release APK: **BUILD PASS** (80.3s). Firmware embedded in APK, source asset and delivered binary: **byte-for-byte match**. Firmware SHA-256: `73713699e53f109d0d40dd98c2d627f9f6dce5719e7a44831d4fe4409575990d`. Exact sizes/hashes are in the package artifact-checks.json.
