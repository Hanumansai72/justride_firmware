# JustRide 2.8-inch firmware engineering audit

Initial audit completed 2026-10-18 before source changes. Existing uncommitted firmware work was present and is preserved.

## Architecture and resources

- Active ESP-IDF 5.5.5 configuration targets **ESP32**, not the ESP32-S3 claimed by the README. Physical module identity still needs inspection.
- Configured flash: 4 MB; two 0x1B0000 OTA slots, NVS, OTA metadata, PHY and a 0x30000 user-image partition. PSRAM is disabled. This does not establish the physical module's installed RAM.
- ILI9341, 320×240 landscape, SPI2 at 40 MHz; XPT2046 shared SPI at 2 MHz. Custom RGB565 rasterizer, polling SPI with DMA enabled; no LVGL or full framebuffer.
- Phone owns location, routing, maneuver state and ride history. Firmware receives compact navigation FF30/FF31 packets and returns FF32 notifications. Separate CTS, image FF20 and OTA FF10 services use NimBLE; no classic AVRCP implementation.
- Phone uses Google Routes v2 TWO_WHEELER, legacy Directions, Places and Google Maps Flutter SDK; existing fallbacks include public OSRM, Photon and Nominatim. No Static Maps or Map Tiles download/transfer exists. Firmware draws a route polyline, not a road basemap.
- Android already exposes MediaSession transport controls through a MethodChannel, but no Dart-to-firmware music integration or device music UI exists.

## Findings and priority plan

1. **Navigation correctness:** asynchronous start/reroute results lack ride generations; stale responses can alter a stopped/new ride. Pause does not stop progress. Failed reroutes restore stale coordinates. Arrival leaves GPS listeners running. Fix lifecycle guards and bounded retry backoff first.
2. **False guidance:** all-provider failure fabricates a direct route; firmware fabricates a missing next turn. GPS filter eventually accepts arbitrarily inaccurate samples. Remove invented guidance, validate accuracy, and use the existing confirmed off-route corridor before rerouting.
3. **Rendering:** same-screen transitions reset the persistent HUD; BLE updates force navigation over other screens. SPI locks cover individual commands rather than complete window/pixel operations. Static fill buffers can race. Fix transaction boundaries, retain screens and update changed regions.
4. **Music:** bridge new bounded BLE messages to existing Android MediaSession APIs, expose actual permission/session state and metadata, add large device touch controls. No optimistic/fake playback. Apple Media Service is a different accessory protocol; current custom Android bridge does not provide iOS support.
5. **Transfer safety:** reject OTA writes exceeding declared image size. Gallery transfer ignores sequence/partial image errors and blocks BLE while erasing/writing flash; preserve working protocol while tightening validation. Gallery uploads must never be repurposed for moving maps because they write flash.
6. **Recovery:** reconnect syncs active phone state, but idle reconnection leaves stale device navigation. Device notification subscription should request authoritative state. Reject queued writes from old connections/rides.
7. **Measurement/testing:** build the configured target, run app regressions and targeted malformed-packet/lifecycle checks; add runtime heap/render instrumentation. No latency, multi-hour stability, electrical, calibration or riding claims without hardware measurements.

## Provider and platform dependencies

Google's general terms §3.2.3(f) restrict embedded vehicle use, §3.2.3(a) restrict extraction, and service-specific Routes terms restrict combining Routes content with a non-Google map. A detachable motorcycle display is not assumed exempt. Commercial deployment needs provider/contract confirmation. Do not scrape phone SDK screenshots, download unofficial tiles, mix Google routes with an OSM basemap, or cache Google content as a workaround.

Supported next step: obtain written approval for the complete phone-to-accessory architecture, or select a routing **and** map provider licensed together for embedded use, including attribution and permitted caching. Then implement a bounded RGB565 region transport and backing layer for that provider. No authorized road-image source is configured here, so road-basemap rendering remains blocked.

Sources checked 2026-10-18:

- https://cloud.google.com/maps-platform/terms
- https://cloud.google.com/maps-platform/terms/maps-service-terms
- https://developer.android.com/reference/android/media/session/MediaSessionManager
- https://developer.apple.com/library/archive/documentation/CoreBluetooth/Reference/AppleMediaService_Reference/Introduction/Introduction.html

Android active-session access requires enabled notification-listener access (or a privileged system permission). The existing manifest declares that listener. Controls depend on actions exposed by the selected player. iOS custom bridge is unsupported; an AMS implementation and device testing would be separate work.

## Verification record

No physical hardware verification has been performed. The following results describe source inspection, host tests and compilation only.

## Changes implemented

| Area | Files and changes |
| --- | --- |
| Ride lifecycle | App `navigation_service.dart`: serialize stop/start, reject duplicate starts and obsolete asynchronous responses using ride generations, invalidate reroutes on pause/arrival/stop, retain latest coordinates after route failure, publish replacement totals/revision, detach GPS listeners on arrival. |
| Guidance and ETA | App `google_maps_service.dart`: reusable HTTP client, eight-second timeout per routing provider, reject missing route geometry/steps, return failure when all providers fail. Remove fabricated direct routes and arbitrary OSRM speed calibration. Navigation progress measures remaining step geometry and retains provider step travel-time estimates. |
| GPS and rerouting | `gps_accuracy_filter.dart`, `off_route_detector.dart`, `location_service.dart`: reject nonfinite accuracy and out-of-order fixes; never promote repeated inaccurate or implausible-jump samples; invalid fixes do not refresh GPS availability. Require a 50 m corridor deviation, accuracy ≤25 m, motion ≥1.4 m/s, three samples and four seconds. Missed-turn heuristics no longer bypass this confirmation. |
| Retry behavior | One reroute in flight; successful reroutes have a ten-second cooldown. Consecutive failures back off 20/40/80 seconds, capped at 80 seconds. Existing provider fallbacks can take up to approximately 24 seconds in total; no unbounded network wait remains in route requests. |
| Route projection | `route_window_transformer.dart`: bounded 800 m forward matching window limits jumps to distant overlapping sections; retain real GPS displacement from the route. Road alignment suppresses compass noise near the route; meaningful off-route displacement uses the supplied heading. |
| BLE transport | App `ble_navigation_service.dart`, firmware `ble.c`: capability negotiation, route revisions, old-connection/old-route write rejection, a 16-operation write bound and latest-only regular telemetry. Phone resynchronizes active navigation/route/context on connection and sends idle state when no ride can be recovered. The device retains the prior view with a syncing/offline indicator and hides stale speed until fresh state arrives. |
| Recovery | A saved destination/session is restored only for its saved account owner. The existing session ID is reused. A restarted phone recalculates its route from a fresh fix; it does not claim to restore an unavailable old route image. |
| Navigation rendering | `display.c/.h`: fixed-size latest-state queues for navigation, geometry, context, media, BLE connection and notifications; BLE callbacks no longer wait for those draws. Same-screen requests retain the HUD. Ordinary telemetry does not steal the selected screen. Touch and renderer share a recursive display lock. Notification expiry no longer draws from the FreeRTOS timer service. |
| SPI and images | `display_tft.c`: lock complete address-window/pixel writes, protect the reusable DMA fill buffer, address each image chunk independently so other draws cannot corrupt the image cursor. Report SPI initialization errors and mutex allocation failure. Gallery upload validates sequence, size, even RGB565 bytes, flash errors and completion. Gallery uploads are rejected during rides/OTA; image and OTA transfers cannot overlap. App `image_transfer_service.dart` reads device completion before reporting success. |
| Music | Firmware `display.c/.h`, `ble.c/.h`; app `ble_navigation_service.dart`, `ble_media_packet.dart`, Android `MainActivity.kt`: previous/play/pause/next/volume touch commands reach the real active Android MediaSession. Return actual state, bounded title/artist and supported actions. Explicit unsupported, permission-missing, no-session and disconnected states. Two-second metadata polling sends changed snapshots only. Device touch commands are rate-limited; a busy bridge drops repeated commands. No album-art transfer or fake playback. |
| Navigation priority | Music is available by swiping between Navigation and Picture. Large 94×60 playback targets and separate volume controls fit the physical 320×240 display. New rides and newly active critical turns return to navigation; ordinary GPS/music updates retain the chosen screen. IMU tilt no longer supplies a false rider bearing or substitutes for navigation instructions. |
| OTA/startup | `ota.c`, `main.c`, `display.c`: bound writes by declared image size, keep dual-slot verification/rollback, check critical startup task/init failures, and initialize idle navigation before enabling BLE so boot cannot overwrite an early ride. |
| Honest diagnostics | Firmware reports unavailable measurements as unknown instead of hardcoded 100% battery/healthy GPS. App `device_settings_screen.dart` reports the diagnostics request result instead of claiming every subsystem passed after a delay. |

## Additive wire protocol

Existing unwrapped FF31 navigation packets with the production 21-byte header remain accepted. FF32 read returns `[0x01, 0x01]` for capability version 1; old firmware returns version zero. The app sends extensions only after reading this capability.

- Versioned navigation: `[0x13, revision_u32_le, original_packet...]`. The wrapper supports navigation lifecycle, route geometry and context packets. Firmware rejects older revisions; app queue operations also carry connection and revision generations. Revision comparison resets on a new BLE connection.
- Media state on FF31: `[0x12, state, actions, volume_percent, title_length, title_ascii..., artist_length, artist_ascii...]`. Maximum 78 bytes, title 40 and artist 32 characters. State values: unsupported 0, permission missing 1, no session 2, paused 3, playing 4, other 5. Action bits: play, pause, next, previous, volume.
- Device commands on FF32: `[0x12, command]`: play 1, pause 2, next 3, previous 4, volume up 5, volume down 6, snapshot request 7. Commands do not optimistically alter playback state.
- Existing clock, navigation, gallery and OTA service UUIDs remain unchanged. Android and firmware updates should be distributed together for music/revision support; older firmware retains legacy navigation only.

## Maps status

The active route uses actual provider road geometry. The device still has **no licensed road basemap**, tile downloader, Google SDK image capture or live-map image decoder. No map image decoding/caching success is claimed. The existing gallery path is a persistent photo upload, not a moving-map transport. It writes flash and is unsuitable for repeated map frames.

The existing fallback stack also needs a commercial provider/provenance/attribution review: OSRM's driving profile is not a verified motorcycle-specific route service, and public community endpoints are not a production SLA. Reusing these existing fallbacks does not establish commercial permission. Provider approval is required before road-map integration or release claims.

## Platform status

Android integration is compiled and uses the enabled notification listener to obtain an active MediaController and the actions that controller advertises. A successful command dispatch is not proof that a particular third-party player executed it. Real phone/player testing remains required.

iOS music is explicitly unsupported by this custom bridge. Apple's documented AMS exposes media control over BLE to an accessory acting as a GATT client; implementing and testing that client is separate from the current custom GATT-server bridge. No claim is made that BLE automatically provides AVRCP or that a Flutter iOS app can control every other music app.

## Automated checks

- Complete Flutter regression suite: **114 tests passed** (106 existing plus eight new regressions).
- New `test/firmware_navigation_bridge_test.dart`: bounded music packet states/metadata, repeated poor/NaN GPS and stale sample rejection, timed deviation confirmation, all-provider/invalid-route failure, duplicate/canceled starts, pause and stale reroute rejection after stop, failed reroute retaining latest GPS with request cooldown, replacement totals and route revision.
- Static analysis of changed navigation/BLE/maps/GPS/image/model/test files: **no issues found**.
- ESP-IDF firmware build and size gate: **passed**. Final sizes are recorded below.
- Android release APK build: **passed**, with the final source. Build warnings concern the installed Java/Android SDK tooling, not a tested runtime failure.
- `git diff --check`: passed for both repositories.

The orientation regression initially failed when raw compass heading tilted an on-route road. It was fixed by retaining road alignment near the route; the full suite was rerun successfully. A test-fixture type error was also corrected before the eight new tests passed. No known failing automated test remains.

## Measurements and remaining verification

Only binary/linker sizes were measured on this host. No frame-rate, input-to-display latency, BLE latency, runtime heap improvement or hours-long stability figure has been measured. The render task now logs maximum draw duration, free heap, largest heap block and its stack high-water mark every 30 seconds for device collection. These are instruments, not claimed results.

Before release, verify on the actual board and phone:

- Physical ESP32 identity/flash and display/touch wiring/calibration; fresh-fix ride start, successive/missed turns, sparse road geometry, arrival and low-speed GPS drift.
- Disconnect/reconnect loops, device power cycling, Android backgrounding/process death, GPS loss/recovery, network failure and slow/invalid routing responses. Restart recovery retains session identity/destination; intermediate ride statistics are not continuously checkpointed and may be lost after phone process death.
- Music permission grant/revocation, no player, unsupported transport actions, changing player, long/non-ASCII metadata, repeated touches and critical navigation turns during playback control.
- Invalid/truncated/out-of-order BLE packets on the actual ATT stack, image flash failures, incomplete uploads, OTA success/abort/disconnect/rollback and low-memory startup. C parsing and SPI behavior have been compiled, not hardware-executed by the Dart tests. The older 18-byte debug-header format is ambiguous for longer strings and is not covered by the production protocol tests; use the existing 21-byte format or negotiated version 1.
- Multi-hour heap/stack/CPU/watchdog measurements and touch/render contention. Polling SPI and per-pixel software text rendering remain potential bottlenecks requiring real measurements.
- Current privileged GATT commands still lack authenticated bonding/access control. A production pairing/security design is required before exposed OTA/restart controls are considered secure.

## Final classification

- **Implemented and host-verified:** route failure handling, GPS quality/confirmation logic, duplicate/canceled start protection, stale reroute rejection, retry suppression, replacement totals/revision and bounded media packet serialization.
- **Implemented, hardware verification required:** incremental device rendering, SPI locking, music control, BLE route revision/recovery, image/OTA safety and startup checks.
- **Partially implemented:** phone-restart recovery retains session/destination but not a continuously checkpointed telemetry history; performance instrumentation exists without on-device measurements.
- **Blocked:** real road-basemap rendering pending a commercially permitted map/routing architecture and attribution requirements.
- **Unsupported:** iOS music in the current custom BLE bridge; no AMS client has been implemented.

## Final build measurements and artifacts

| Measurement | Result |
| --- | --- |
| Baseline app binary | 646,160 bytes |
| Final app binary | 650,160 bytes |
| Binary growth | 4,000 bytes |
| Smallest OTA slot | 1,769,472 bytes |
| Remaining OTA slot space | 1,119,312 bytes |
| Linker IRAM | 117,027 / 131,072 bytes (89.28%); 14,045 bytes remain |
| Static linker DRAM | 30,484 bytes; data 22,396 and BSS 8,088 bytes |
| Android APK | 60,653,678 bytes |

Linker DRAM availability is not a runtime free-heap measurement. Runtime memory, CPU and latency have not been benchmarked. The increase in binary size includes fixed queues, error checks, protocol handling and the music UI; no full-screen framebuffer or image-decoder dependency was added.

- Firmware target: configured **ESP32**, ESP-IDF 5.5.5, 4 MB flash. Verify physical module identity before flashing.
- [Firmware binary](C:/Hanumansai_Projects/Justride/justride_firmware/build/justride_firmware.bin)
- Firmware SHA-256: 3e5d06fa862ffb06caba233f28f0bae5983d2008110c7b86c2002a4066c1d998
- [Android APK](C:/Hanumansai_Projects/MobileApp-main/MobileApp-main/downloads/JustRide-firmware-bridge-20261018.apk)
- APK SHA-256: 07f27dc891a6692fd892a2b29177e0d90943d2f1d0c544644f3f10e3de31ca44
- Firmware build log: build/audit-build.log
- App logs: build/firmware-navigation-tests.log, build/firmware-navigation-analysis.log, build/firmware-final-analysis.log, build/firmware-navigation-apk.log

The firmware was **not flashed** and the APK was **not installed on a physical phone**. These are compilation artifacts for hardware verification, not a certification of production readiness.
