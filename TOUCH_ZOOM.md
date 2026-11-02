# Touch-only zoom: 2.8-inch version-2.0

Target confirmed by the user: the existing ESP32, 320x240 landscape ILI9341
and XPT2046 resistive panel. This is not the 466-pixel round-display port.
Both firmware and Android source changes are required; no new dependencies.

## Controls

- Double-tap the unobstructed map area to zoom in one level.
- Tap, then press again nearby and hold: drag upward to zoom in, downward
  to zoom out. Each 32-pixel step selects one level, at most once per 120 ms.
- The second press must start within 320 ms and 20 pixels of the first tap.
  The first tap must be short (280 ms including release filtering) and move
  no more than 12 pixels. Long holds and widely separated taps do not arm zoom.
- Ordinary swipes retain screen/brightness behavior. Music, notifications,
  calls, modal dialogs, the trip card and compass do not arm zoom. Context
  loss during an armed drag consumes that press until release.
- Existing compass tap returns to follow. The existing trip card toggles
  overview. No zoom buttons, toolbar or new visible controls are added.

The XPT2046 reports one resistive X/Y contact, not separate finger contacts.
Pinch and two-finger tap are deliberately not implemented. Controller evidence:
[XPTEK datasheet, general description and digital interface](https://files.waveshare.com/upload/9/98/XPT2046-EN.pdf).
Host gesture replay is not a physical pressure/noise or glove-use trial.

## Camera and real geometry

The phone owns projection and route-window selection. Four follow levels use
up to 150, 300, 700 and 1800 meters of actual route. The closer two retain the
existing maneuver-focused approach camera. Follow keeps the rider at the
existing lower map anchor. No bitmap is enlarged to claim extra street detail.

Level five fits the entire remaining route into a north-up viewport padded
clear of the existing trip card, compass, attribution and marker radius.
Its viewport stays fixed during GPS progress; the rider marker moves in it.
Rerouting recomputes its bounds. Zooming back in returns to follow. There is
no timer that changes the user's selected zoom. Overview uses a small white
rider dot and a green destination marker. The rest of the HUD is unchanged.

The default Android build has `GOOGLE_DEVICE_MAPS=false`: it sends actual
route polylines, not street tiles. The existing compact protocol carries at
most 14 sampled route points from the phone (16 supported by firmware).
Long/very curvy route-only overviews are therefore simplified, not a detailed
representation of every flyover/corner. Do not treat their line as lane-level
geometry. With authorized Google static imagery enabled, the native-size
phone-rendered overview retains all route vertices rather than thinning them.
No unrelated roads are invented when imagery is unavailable.

Static-map mode is discrete zoom, not smooth continuous vector-map zoom.
Requests are debounced 600 ms and remain subject to the existing provider
refresh interval (10 seconds minimum, quota/backoff may increase this).
The previous valid map stays visible until its replacement is complete.
No offline/persistent image cache or extra provider is introduced. Existing
branding/copyright rows remain untouched. Provider account permissions and
deployment terms still need owner review; the opt-in flag was not enabled.
See [Google terms: attribution and caching](https://cloud.google.com/maps-platform/terms)
and [service-specific terms](https://cloud.google.com/maps-platform/terms/maps-service-terms).

## BLE and bounded memory

- Reuses device status command `0x14`: numeric map zoom 17..14, 255 overview,
  254 legacy recenter. Firmware sends the latest selected level after 250 ms
  of gesture quiet; limits produce no additional zoom request. Reconnection
  resynchronizes the chosen camera after phone state becomes ready.
  Local BLE enqueue failures retry every 500 ms without blocking the touch task.
- Route packet `0x0B` keeps its layout. Previously unused flag bit 5 denotes
  overview and bit 6 denotes phone-projected geometry. The latter prevents
  scaling twice. Legacy packets keep the old local scaling behavior.
- Android coalesces pending route writes and cancels obsolete map transfers
  when the camera generation changes. Existing navigation/connection revision
  guards are preserved; newer route data is not replaced by an older frame.
- Raster reception uses a retained front buffer and a separate incoming buffer.
  Only a complete CRC-checked image swaps into view. Failed/aborted transfers
  keep the last map, including notification-region restoration and GPS marker
  updates. Duplicate ATT chunks/commits are idempotent; stale frame IDs are
  rejected with wraparound-aware comparison. New connections reset the peer's
  frame-counter session without clearing the visible map.
- Two buffers are allocated lazily only when raster maps are used: at most
  160,008 image bytes total. A 32 KiB free-heap reserve is required before each
  allocation; allocation failure rejects the transfer without losing the front
  frame. This extra 80,004-byte buffer is the explicit cost of retaining a fully
  readable old map while decoding a replacement. Runtime heap suitability on
  the actual board remains unverified. Route-only mode needs neither buffer.
- Ten seconds without incoming frame data aborts that transfer, not the user's
  camera choice. A first incomplete map does not suppress vector-route drawing.

## Changed files

Firmware repository:

- `main/main.c`: single-finger gesture recognition.
- `main/display/display.h`, `display.c`: gesture gating, level limits,
  coalesced requests and phone-projected overview rendering.
- `main/ble/ble.h`, `ble.c`: the two route-view flags and notification enqueue result.
- `main/display/map_frame.h`, `map_frame.c`: atomic frame retention/session guards.
- `tools/test_touch.py`: gesture and original touch/rotation regressions.
- `tools/test_zoom.py`: production receiver/camera host checks and UI capture.
- `README.md`, `TOUCH_ZOOM.md`: usage, limitations and verification.

Android repository (`C:/Hanumansai_Projects/MobileApp-main/MobileApp-main`):

- `lib/core/services/navigation_service.dart`: camera ownership and fixed overview.
- `lib/core/utils/route_window_transformer.dart`: real wider route windows/overview.
- `lib/core/models/device_map_frame.dart`: shared Mercator inverse and padded fit.
- `lib/core/models/ble_navigation_packet.dart`: route flags.
- `lib/core/services/device_map_service.dart`: debounce/overview exit/reroute handling.
- `lib/core/services/ble_navigation_service.dart`: command handling and latest-only writes.
- `test/touch_zoom_test.dart`: framing, bounds, projection and packet tests.

Existing unrelated app changes, deleted tests and the modified embedded firmware
asset were preserved. The APK's embedded firmware asset was NOT replaced: flash
the newly built firmware separately. No hardware was flashed or app installed.
Changes remain uncommitted on the current repositories.

## Verification and reproduction

28 October 2026 software checks:

- ESP-IDF 5.5.5 build of the 2.8-inch firmware passed; 59% OTA slot headroom.
- Touch driver plus real main.c FSM: 98 checks per rotation, 392 total, passed.
- Map receiver/camera: allocation failures, retained front pixels during receive,
  corrupt/incomplete commit, duplicate chunks/commits, old IDs, ID wrap, idle
  transfer expiry, peer counter reset, zoom limits and coalescing deadline passed.
  Failed/timed-out frames can restart with the same ID; a committed frame's
  older duplicate cannot replace a newer one.
- Android: 41 targeted zoom/navigation/maneuver/media tests passed.
- Static analysis of the six changed Dart source files plus zoom tests passed.
- Android release APK build passed (58.6 MB); embedded firmware asset preserved.

Run `python tools/test_touch.py`. From the Android repo run `flutter test
test/touch_zoom_test.dart` with `JR_ZOOM_FIXTURE_OUT` set to this firmware repo's
`build/zoom-tests/packets.json`, then run `python tools/test_zoom.py` here.
Both host tools require MSVC Build Tools; the capture tool also requires Pillow.

`build/zoom-tests/touch-zoom-comparison.png` shows detailed, intermediate and
overview frames from actual Android fixture packets and production C drawing
code. Coordinates are explicitly test fixtures, not live GPS or downloaded
street-map evidence. Native screen captures are the adjacent `zoom-0.png`,
`zoom-1.png`, `zoom-2.png` files. The comparison is scaled for readability.

Unverified: physical gesture reliability/gloves, SPI/render latency during BLE
traffic, heap headroom with two raster buffers, sustained live GPS/rerouting,
network quota behavior, imagery availability, and on-road/flyover correctness.
Perform these checks on a stationary test rig before rider use.
