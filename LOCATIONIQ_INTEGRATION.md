# LocationIQ integration audit — 2026-10-18

## Before implementation

The configured target is classic ESP32, 4 MB flash, PSRAM disabled. Physical module identity remains unverified. ILI9341 uses SPI2 at 40 MHz, DMA-backed polling writes, landscape 320×240. XPT2046 touch shares SPI at 2 MHz. Custom renderer, no LVGL/full framebuffer. Navigation map rectangle is x=122,y=4,width=198,height=194.

There is no existing Google Static Maps fetch/transfer pipeline to replace. `google_maps_service.dart` calculates Google Routes/Directions with public OSRM fallback and provides Places/geocoding. `navigation_service.dart` owns GPS, filtered progress, confirmed off-route detection, cooldowns, ride generations and recovery. `ble_navigation_service.dart` serializes FF31 writes and negotiates FF32 capabilities. Firmware `ble_nav_data_access` parses navigation; `display_nav_render_partial` / `draw_hud_road_corridor` draw a vector route without roads. Gallery FF20 writes flash and must not carry repeated map frames. OTA FF10, media FF31/32, touch, notifications and ride lifecycle must remain intact.

## Decision

Add authenticated backend LocationIQ static-image proxy with shared Redis rate/budget controls. Keep credentials server-side; never place the token in the APK. Reuse the existing HTTP, routing parser, BLE queue and RGB565 TFT writer. Add capability version 2, bounded RAM frame transfer, dimensions/sequence/CRC validation, and atomic acceptance. No new firmware HTTP, flash image cache or graphics dependency.

LocationIQ mode uses existing OSRM driving routes exclusively. Google routing remains available in Google mode; no Google geometry/POIs/instructions are placed on LocationIQ images. Public OSRM is a prototype dependency, not a motorcycle profile or commercial SLA. Use hosted/self-hosted OSRM, Valhalla, GraphHopper or contracted LocationIQ routing before production scale. OSRM reuse needs least change; Valhalla/GraphHopper offer richer profiles but require separate integration. LocationIQ routing shares account quota.

Use native-size images and local projected rider updates. Refresh on viewport edge, route/style/zoom change with minimum spacing, one request in flight, stale response rejection and bounded retry. Retain displayed pixels during downloads/transfers; do not claim measured smoothness. RAM is constrained: one 76,824-byte backing frame, no second framebuffer; a screen change during incomplete replacement may require a fresh frame. Preserve attribution and add the free-plan LocationIQ backlink in the companion.

Backend deployment, shared Redis, provider token restrictions and physical-board/phone measurements remain release prerequisites. No hardware throughput, runtime heap, touch latency, GPS latency or long-duration performance has yet been measured.

Verified references: https://docs.locationiq.com/docs/static-maps-api-reference ; https://locationiq.com/pricing ; https://locationiq.com/attribution . Free plan currently lists 5,000/day, 2/second, 60/minute account-wide; budget must reserve usage outside this endpoint. Keep images only in current-session RAM; no disk or server-side map cache.

## Implementation report — 2026-10-19

### Files changed for this integration

Paths below are relative to their repositories. Existing dirty audit changes in these repositories were preserved; this list describes the LocationIQ additions, not ownership of every working-tree diff.

| Repository | Files / responsibility |
| --- | --- |
| Backend | `src/services/staticMap.js`: fixed native viewport, validated provider parameters, native fetch with full-request timeout/byte bound, JPEG signature/dimensions, shared account quota reservation. |
| Backend | `src/routes/mapRoutes.js`, `src/app.js`: authenticated USER-only `GET /api/maps/static`, existing user limiter, safe errors, no-store response, upstream Server-Timing. `.env.example`: token/budget names; real token only in ignored `.env`. `test/staticMap.cjs`: runnable Node built-in tests. |
| Companion | `lib/core/constants/device_map_config.dart`: map mode, viewport and endpoint; configurable minimum refresh. `lib/core/services/map_image_provider.dart`: centralized provider interface and authenticated binary proxy client. |
| Companion | `lib/core/services/device_map_service.dart`: one download in flight, viewport reuse, zoom/recenter/overview, speed-dependent zoom, debounce, stale-request cancellation/rejection, exponential retry, native JPEG validation/decoding, compatible route rasterization, RGB565/RLE/CRC. |
| Companion | `lib/core/models/device_map_frame.dart`: Web Mercator projection, CRC/RLE, native metadata, MTU-bounded chunks, protected marker coordinates. `lib/core/services/ble_navigation_service.dart`: capability v2, serialized acknowledged chunks, bounded retry, connection/ride guards, frame replacement and status. |
| Companion | `lib/core/services/google_maps_service.dart`: explicit route provenance; OSRM-only in LocationIQ mode, Google implementation retained in separate Google mode. `lib/core/services/navigation_service.dart`: suppress Google POI/city context in LocationIQ mode. |
| Companion | `lib/features/navigation/widgets/device_map_view.dart`, `lib/features/navigation/screens/active_navigation_screen.dart`: matching map provider, local rider marker, status and provider backlink, zoom/recenter/overview controls. `lib/main.dart`: initialize map coordination. `maps_loader_stub.dart`, `maps_loader_web.dart`, Android `MainActivity.kt`: fixed attribution link through native/browser APIs without a dependency. `test/device_map_test.dart`: projection, provenance, codec, protected copyright strip, MTU, quotas/errors, CRC/RLE. |
| Firmware | `main/display/map_frame.c/.h`: one lazy 76,824-byte buffer, minimum 32 KiB remaining heap requirement, RLE validation, frame identity, sequence/duplicate checks, CRC validation and deferred rendering. `main/display/display.c/.h`: integrate under UI lock, move speed/status outside map, partial marker patch restoration, keep old LCD pixels while receiving. |
| Firmware | `main/ble/ble.c`: v2 capability and revision-scoped map operations; reject map writes during gallery/OTA. `main/CMakeLists.txt`: compile new module. Companion `assets/firmware/justride_firmware.bin`: matching firmware for existing OTA flow. |

### Wire format

FF32 capability read is now `[1,2]`. Old firmware receives no map packets. FF31 map operations use the existing `[0x13, revision_u32_le, inner...]` wrapper. Multibyte metadata is little-endian; pixel pairs are big-endian RGB565.

| Inner operation | Fields |
| --- | --- |
| 0x14 START | frame u16, width u16, height u16, decoded-pixel CRC32 u32; 11 bytes total |
| 0x15 DATA | frame u16, sequence u16, triples `(count 1..255, pixel high, pixel low)` |
| 0x16 COMMIT | frame u16; exact 76,824 decoded bytes and CRC must match |
| 0x17 ABORT | frame u16 |
| 0x18 POSITION | frame u16, x u16, y u16, heading-degrees u16 |
| 0x19 STATUS | state u8: loading 0, accepted 1, error 2, quota 3, viewport edge 4 |

MTU chunks reserve ATT/header/revision overhead and keep RLE triples intact. Every map write requests ATT acknowledgement; errors never fall back to unacknowledged writes. At most one chunk is queued by the map sender at a time, sharing the existing bounded queue with navigation/music. An identical immediately previous chunk may be retried; altered duplicates, wrong sequence/dimensions/frame or overflowing decoded size are rejected. Commit acknowledgement confirms firmware validation, not that a physical LCD was observed.

GPS updates use the downloaded frame's exact north-up Web Mercator projection. Firmware restores only the previous 21×21 marker patch and draws its replacement, protecting the bottom attribution strip. It blits a complete new map only after commit; this is validated replacement with no partial image display, not double-buffered scanout. During replacement, the single buffer cannot also hold the old image for restoring markers: the previous LCD map/marker is retained until the new frame is complete. Leaving and returning to navigation during an incomplete frame can temporarily show the vector route. No flash map writes or ESP32 HTTP requests were added.

### Request policy

- Default minimum map spacing: 20 seconds, configurable with `DEVICE_MAP_MIN_REFRESH_SECONDS` (clamped 10–300). This is a ceiling of 180 requests per continuously moving rider-hour at the default, **not a measured usage figure**. Stationary/in-viewport fixes cause no downloads.
- Refresh on initial map, viewport margins, route revision, requested camera/zoom change. Automatic zoom uses speed thresholds with hysteresis; explicit zoom/overview disables that until recenter. Overview geometry is downsampled to bound painting work. Static maps remain north-up.
- One HTTP request and one retained completed frame. Cancel obsolete requests on stop/route/camera changes and reject late results again after preprocessing. Retain valid imagery during API errors/rerouting and retry with bounded backoff; 429 Retry-After is respected conservatively.
- Shared backend Redis sliding limits: 2/second, 60/minute, configurable rolling 24-hour budget (default 4,500, hard maximum 5,000). Reserve before upstream requests; failures count conservatively. Production fails closed if shared Redis is unavailable; existing development-only memory store is explicitly not distributed.
- All app consumers share this proxy. Requests made elsewhere with the same account are outside its counters; keep a budget reserve and monitor the LocationIQ dashboard. No separate per-device 5,000 allowance is assumed.

### Verification and measured data

| Check / metric | Result |
| --- | --- |
| Direct live backend-client request | HTTP 200; 198×194 JPEG, 4,329 bytes, 1,161 ms for one sample. Earlier Python-client attempt returned 403/1010. |
| Host preprocessing | Observed 75–135 ms across test runs for the same sample; latest full-suite run 135 ms. These are desktop Flutter test measurements, not a phone benchmark or claimed improvement. |
| Transfer representation | 76,824 RGB565 bytes → 33,699 RLE bytes for that image plus a synthetic test route. Actual journeys will vary; worst RLE expansion is bounded at 115,236 bytes. |
| Flutter regression suite | 121 passed with LocationIQ enabled. Google mode preserved through an explicit successful mocked Google Routes test; earlier 119-test regression run also passed with LocationIQ disabled. |
| Backend tests | 5 passed: validation, account-wide quota, safe status handling, MIME/size/dimension corruption, timeout/offline/Redis errors. |
| Targeted final analysis | No issues found. Whitespace checks passed. Token scan found no token in tracked/nonignored files. |
| Firmware compilation | ESP-IDF 5.5.5 / configured ESP32 build and size gate passed; 652,608-byte binary. IRAM 117,027 / 131,072 bytes. Static DRAM 30,524 bytes (22,396 data, 8,128 BSS). Dynamic image allocation is additional. |
| BLE throughput / transfer time | Not measured on hardware. App logs compressed bytes and acceptance time; firmware logs transfer duration. |
| ESP32 RLE decode / render time | Not measured on hardware. Decoding occurs per validated chunk; firmware logs full-map blit time. |
| Runtime free heap / PSRAM | Not measured. Configured PSRAM disabled. Runtime heap/largest-block instrumentation exists; link-time DRAM is not free-heap evidence. |
| GPS marker latency / rerouting latency | Not measured physically. Projection/lifecycle/off-route host regressions passed; existing phone navigation state machine remains responsible for rerouting. |
| Touch responsiveness / long rides | Not measured physically. UI/SPI locking and bounded transfers compile; no smoothness or multi-hour stability claim. |

Existing clock, gallery, media, notifications, OTA, navigation UUIDs and ride history remain. Phone/UI regressions and compilation passed; they do not establish real hardware regression success. Firmware navigation-camera functions send phone feedback; actual touch wiring/gestures must be verified on the board. Existing photo-gallery controls were not repurposed for maps.

### Deployment and production prerequisites

1. Deploy this backend revision to `backend.justride.cc`, with `LOCATIONIQ_API_KEY` supplied through the deployment secret environment. The local ignored `.env` does not configure the remote deployment. Configure shared Redis and `LOCATIONIQ_DAILY_BUDGET` (default 4,500). Restrict the provider token to the backend's supported egress configuration.
2. Endpoint: authenticated `GET /api/maps/static?latitude=...&longitude=...&zoom=...&style=dark`. No token goes into the URL sent by the app. This source change has not been published/deployed automatically.
3. Build-time app options: `LOCATIONIQ_DEVICE_MAPS=true` is default; `DEVICE_MAP_ENDPOINT` can select a development proxy; `LOCATIONIQ_DEVICE_MAPS=false` retains the separate Google implementation. Never compile the LocationIQ token into Dart/Android.
4. Install the companion and matching firmware together for map capability v2. Older firmware retains legacy navigation without road imagery. Physical board identity must be checked before flashing; no device was flashed or phone APK installed here.
5. Verify real BLE MTU/throughput and heap with the intended phone/ESP32, interruptions, malformed packets, touch, music, notifications, GPS loss, reroutes, power cycling, OTA/rollback and multi-hour rides. Twenty-second image spacing and acknowledgement performance require road-speed/viewport calibration; when the rider leaves coverage the device reports MAP EDGE and retains the old image rather than inventing a position.
6. Replace/contract the public OSRM driving demo before production load and confirm the intended motorcycle profile. Google imagery/routes remain separate. Review existing Photon/Nominatim fallback usage and commercial support requirements. LocationIQ free use requires the visible companion backlink and map attribution; both are retained. Confirm product-specific licensing if distributing commercially.
7. Existing APK release signing uses the project's debug signing configuration; this remains a prototype artifact, not a store-ready production release. Existing unauthenticated custom GATT controls still require a separate production pairing/bonding/security review.


## Final delivery artifacts

- Android release build passed with final sources. APK: `C:/Hanumansai_Projects/MobileApp-main/MobileApp-main/downloads/JustRide-locationiq-prototype-20261019.apk` — 59,707,510 bytes. SHA-256: `ed31cf2dcd6e1c156bd5da8b8b52819d4fa1d1de0cf1b61bf2251c3f4b02f269`.
- Firmware: `C:/Hanumansai_Projects/Justride/justride_firmware/build/justride_firmware.bin` — 652,608 bytes. SHA-256: `42de52830a39c3852d71b89cb379b1b710551004bd745238d28c9c08ce910a97`.
- APK ZIP integrity checked; bundled firmware matches this build byte-for-byte. Provided LocationIQ token absent from every APK member and tracked/nonignored source file.
- Final full-suite rerun: 121 Flutter tests passed; five backend checks passed; final targeted static analysis: no issues. Source whitespace gates passed.
- Logs: companion `build/locationiq-apk-final.log`, `build/locationiq-tests-final.log`, `build/locationiq-analysis-final.log`; firmware `build/locationiq-build.log`.
- Source and artifacts remain local. No GitHub push, remote deployment, device flash or phone installation was performed in this integration task.
