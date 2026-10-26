# 2D reference HUD implementation — 2026-10-19

## Scope and hardware audit

The supplied photographed design is implemented at the existing panel's native **320×240 landscape RGB565** resolution. The user explicitly approved a **2D map with the same layout**. The panel is ILI9341 over 40 MHz SPI on classic ESP32. PSRAM is disabled. Flash is 4 MB with existing dual OTA partitions; partition layout and OTA protocol are preserved.

The renderer remains the existing custom TFT primitive pipeline, protected by the recursive UI mutex. Navigation, route, media, notification, connection and call updates enter the render task through bounded FreeRTOS queues. No LVGL, full-screen RAM framebuffer, new graphics dependency, or network operation was added to firmware rendering. Fonts are generated Apache-2.0 Roboto bitmaps (8/10/12/16/22/28/36 px), stored as const flash assets; glyph rows use 64 bytes of reusable scratch memory. Existing non-navigation screens retain their existing fonts.

## Native layout

| Component | Rectangle / center |
| --- | --- |
| Speed card | x3 y5 w88 h82 |
| Speed limit | center77,24 radius10 |
| Petrol Bunk | x3 y93 w90 h35 |
| Bike Repair | x3 y132 w90 h35 |
| Map viewport | x94 y0 w226 h177 |
| Latest notification | x100 y4 w135 h27 |
| Trip summary | x245 y4 w73 h34 |
| Compass | center303,150 radius12 |
| Maneuver banner | x3 y177 w314 h59 |
| Maneuver icon | x7 y183 w67 h49 |

The compass is slightly higher than the photographed reference to avoid covering the real provider attribution. The map is flat north-up geometry, with a follow-camera offset that leaves more road visible above the rider. Route lines, chevrons and POI positions are projected from actual coordinates; there are no invented flyovers, buildings, repair shops, speed limits or POI distances. POIs absent from the lookup show `-- km`; the speed-limit badge shows `--` because the current navigation protocol supplies no road speed-limit data. Unknown or disconnected speed shows `--`.

The bottom banner shows only the current maneuver. The existing next-maneuver fields remain compatible on the wire but are not rendered. Instructions and distances use actual navigation state; `TAKE FLYOVER` is not hardcoded. Speed filtering/calculation and GPS heading calculation are preserved.

## Integration and behavior

- The app requests a 226×177 LocationIQ map through the existing authenticated backend proxy. The backend accepts only the existing 198×194 size or the new 226×177 size; existing size remains the default. Rate limits, authentication and business logic are unchanged.
- Dark map raster contrast is adjusted toward navy for display readability. The provider attribution strip is copied without recoloring; route lines and POIs do not cover it. LocationIQ and OpenStreetMap attribution remains on screen.
- Firmware advertises navigation protocol revision 3 with the existing motion bit and a HUD extension bit. It accepts both frame sizes. Legacy maps are centered/cropped without stretching, and their original attribution strip is preserved. The new app sends wide frames only to revision 3 firmware, preventing invalid frames being sent to old firmware.
- Both POI types are retained independently and replayed after reconnect. Existing optional lookups are throttled to 10 minutes/1 km, with nearby cached distances published every 5 seconds while riding. Candidate selection still requires proximity to the rider and active route.
- The existing Hardware Settings page now has an explicit **Notifications on device** switch, default off and persisted locally. Forwarding also needs Android notification access. User-initiated custom alerts are still allowed.
- Legacy notification title/message fields are preserved, with optional app-label metadata for revision 3. UTF-8 packet fields are limited by bytes without splitting code points. The HUD uses ASCII bitmap glyphs; unsupported characters are placeholders.
- Ordinary alerts use the small top-center card and the existing 2-second timer. Navigation continues during alerts. Dismissal restores the covered map region rather than clearing the navigation screen. During an in-flight map transfer, the map backing buffer is unavailable; the covered strip is cleared until the next committed map arrives.
- Incoming Android telephony calls use a separate full-screen overlay, with caller information when Android supplies it. Navigation/map queues continue processing. Only supported Accept/Decline actions are enabled. Requests do not pretend the call ended: the real phone-state callback restores the underlying screen. On Android 10+, decline remains disabled unless the app qualifies as the default dialer. This implementation does not add a dialer role or new phone permissions. VoIP call lifecycle/control is not integrated with Android telephony.
- Compass tap recenters the real map. Trip-summary tap requests route overview. Old invisible touch targets from the previous layout were removed.

## Verification and measurements

- ESP-IDF 5.5.5 firmware build: **PASS**, including size report. No compiler errors/warnings in the final build log.
- Image size reported by ESP-IDF: **722,941 bytes** (binary may include padding).
- Static DRAM: **33,428 bytes**; linker-reported unused DRAM **91,152 bytes**. These are static/linker numbers, not runtime free heap.
- IRAM: **117,027 / 131,072 bytes**; remaining **14,045 bytes**.
- Dynamic map backing buffer: **80,004 bytes**, up **3,180 bytes** from 198×194. Allocation requires an additional 32 KiB of reported free heap. The buffer is reused across rides. Marker background cache is 882 bytes.
- 40 MHz SPI theoretical payload-only map transfer floor: approximately **16.0 ms**. This is a calculation, not a hardware measurement; command overhead and drawing take additional time.
- Local backend live map download: **HTTP 200**, decoded JPEG exactly **226×177**. Original check completed in 713 ms. No production server setting or secret was changed.
- C-generated host framebuffer captures: navigation, notification and incoming call. These compile the production `hud_ui.c` and actual `display_tft.c` drawing primitives using a host SPI sink with asserted address-window bounds. They are **not screenshots captured from a physical ESP32**.
- The preview uses a real LocationIQ map and a public OSRM route around Hyderabad (rider sample17.385,78.4867; camera17.385737,78.4867; zoom16). Preview HUD values52 km/h,400 m,11 min,8 km and the sample sender are visual fixtures, not a live ride. Map compositing for the host capture is reproduced with Pillow; production compositing uses Flutter Canvas.
- Runtime instrumentation logs maximum/average rendering time, render ticks/window duration, notification state, free heap, largest block and render-task stack headroom every30 seconds. Map acceptance logs heap and transfer time; map blits log transfer duration. Render task target remains20 Hz. **Actual device FPS, heap, stack, touch latency, BLE reliability and prolonged operation have not been measured without connected hardware.**
- No new test or use-case files were added.

## Visual comparison and limitations

`build/capture/comparison.png` compares the photographed screen rectified to320×240 with the generated C HUD. Additional native and3× captures are in that directory.

This is not claimed pixel-perfect: the approved map is2D; data and roads differ from the reference; bitmap glyphs and small vector icons approximate the photographed typography/icons; flat panels omit the original rendered glow/texture; unavailable data remains unavailable; map copyright occupies real space and requires the small compass adjustment. At320×240, previews and long instructions are width-limited to avoid overlap.

Physical acceptance remains required: flash the firmware, install the matching APK, check actual location/heading/speed, both POIs, every maneuver, notification timeout/restore, call accept and supported decline, Home/switch-app BLE continuity, disconnect/reconnect, restart/resume, touch and a prolonged ride. No physical flashing or live phone control was performed by this task.

## Affected source files for this HUD task

Firmware: `main/CMakeLists.txt`; `main/ble/ble.c`; `main/display/display.c`, `display.h`, `map_frame.c`, `map_frame.h`; new `hud_ui.c`, `hud_ui.h`, font assets/license; font-generation and host-capture preparation utilities in `tools/`.

App: `lib/core/constants/device_map_config.dart`; `lib/core/models/ble_navigation_packet.dart`, `device_map_frame.dart`; map provider/compositor, navigation POI context, BLE navigation, notification and telephony services; existing Device Settings screen; Android `MainActivity.kt`; bundled firmware binary.

Backend: `src/services/staticMap.js` only for the new validated viewport size.

Earlier uncommitted sensor, swipe, background BLE and test-file-removal work is preserved; it is not attributed to this layout change.

## Rollout

Deploy the backend viewport change before using the new wide-map APK; install/flash revision3 firmware with that APK. An older backend returns the legacy size and the new app deliberately rejects the unexpected size. Static map images refresh no faster than the existing20-second minimum (longer on quota/error backoff). Marker/status writes share the serialized BLE queue; image upload is not a continuous animated map stream. The single firmware map buffer retains the last visible image while receiving its replacement; marker packets for an uncommitted frame are rejected. Speed, current maneuver and route progress continue independently. Physical map-transfer smoothness remains an acceptance check.

The existing backend environment still needs a valid LocationIQ key and the configured rate-limit store. No key is bundled in the APK or firmware. Source has not been pushed or deployed by this task.

Build/capture logs: `build/hud-build-final.log`, `build/capture/`. Companion build/analysis results are recorded in the task completion notes below.


## Companion verification

Changed Dart service/model analysis: **No issues found** (including the final compositor and marker-bound changes). The existing Device Settings file also reports an older unused optional parameter and style notices; the new switch compiles in the release APK. `node --check src/services/staticMap.js` and repository whitespace checks pass. Android native code and Flutter release compilation pass. The final APK is checked against the final firmware by comparing its embedded firmware SHA-256 with the built binary and the source asset before delivery.

The Windows host lacks a full Windows SDK. Capture compilation used MSVC with temporary minimal CRT declarations/import libraries in ignored `build/capture/`, retaining the real production drawing code. No system toolchain or SDK was modified. The preparation utility can also be compiled normally on a machine with a complete SDK.

Final package: release APK build **PASS** (34.5 s incremental packaging), firmware binary **723,056 bytes**, embedded/source/delivered firmware byte comparison **PASS**. Firmware SHA-256: `ac1d5e62bd3665fe956d9f3d078371df44c5076f55185558a28a9d010da426ba`. Artifacts are in `downloads/JustRide-2D-HUD-20261019/`, with `artifact-checks.json` for exact sizes and hashes.
