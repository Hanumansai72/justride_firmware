# Verification — 4 November 2026

Software verification passed before pushing this branch.

| Check | Result |
|---|---|
| ESP-IDF 5.5.5 / ESP32-S3 build | Pass; application 821,296 bytes, 54% OTA-slot space free |
| ESP-IDF 5.5.5 / original ESP32 2.8-inch build | Pass; application 769,328 bytes, 57% OTA-slot space free |
| Actual QSPI transport with mocked SPI/GPIO | Pass for SH8601 and CO5300 IDs/offsets, fragmented writes, even-row DMA and unchanged-row suppression |
| Actual FT3168 driver with mocked I2C | Pass for contact mapping, release and read failure |
| Round C renderer and geometry | Pass; all preview screens generated from the production functions |
| Existing XPT2046/gesture suite | Pass; 98 checks × four rotations, 392 total |
| Existing map receiver and camera bounds | Pass |
| Application compiler checks | Pass; all application sources cross-compiled by ESP-IDF |
| Git whitespace check | Pass |
| Power/turn math and wire format | Pass; VCC ×3, USB hysteresis, unknown charge/percentage, golden packets, monotonic estimate, upright/flat turn projection and signed wrap |
| Updated APK focused analysis | Pass with informational style lints; no errors or warnings in changed files |
| APK power/gyro, turn vibration, GPS freshness, pause flag, navigation packets, OTA chip guard, touch zoom and clock tests | Pass; 32 tests |
| Android release APK | Pass; Flutter `build apk --release --no-pub`, approximately 59 MB |

The companion APK source is in the separate MobileApp checkout; no private app
source is published in this firmware repository. Its power card was widget-tested
for live battery/USB changes and disconnect clearing. This is mocked telemetry,
not a physical BLE session. The APK includes a separate S3 firmware asset and
refuses OTA images for the wrong ESP chip.

The round preview fixture checks that blink off clears only the chevron rectangle:
speed, maneuver distance, trip footer and other pixels stay unchanged. Both
directions are captured on/off, with 466×466 two-frame GIFs (500 ms per frame).
Recovery tests cover five-second packet expiry, disconnect, sync, weak GPS,
rerouting, pause and automatic return to live state. Turn alerts are suppressed
in every recovery state and after maneuver completion. Power warning tests cover
five stable low samples, one-shot notification, hysteresis and invalid/USB readings.
APK tests mock native haptics to verify opt-in, one alert per maneuver and no alert
for an old fix. The wire-level pause bit is checked for update and sync packets.

The wider existing app suite is not clean: `speed_filter_test.dart` calls a missing
`SpeedFilter.toKmh`; the optional `ui_cleanup_test.dart` run failed on unsupported
desktop Bluetooth, navigation settling and an outdated `.webp` asset expectation
(the current product image is `.png`). Those unrelated tests were left intact.
CodeRabbit was unavailable on this Windows host; automatic approval review also
rejected its remote installer. No CodeRabbit review is claimed.

The S3 build uses 16 MB flash and 8 MB octal PSRAM. The retained ESP32 build uses
4 MB flash and excludes the Waveshare touch/IMU/panel paths.
Neither profile exceeds its original 0x1B0000-byte application OTA slot.
The original 2.8-inch working checkout remains on version-2.0.

The first full local build encountered an internal compiler error in an unused
ESP-IDF RGB driver. The project now builds its declared component graph from
`main` rather than every installed optional component. The successful final
build used the installed compiler directly, without the failing cache launcher.
No SDK source, compiler binary or driver dependency was patched to obtain a pass.

## What was not tested

No physical board was flashed or exercised. Real display output, touch orientation,
QMI8658 axis alignment, power consumption, BLE range/pairing, restart timing and
OTA interruption/rollback still need a bench run. UI images use explicitly
synthetic telemetry; they are not photographs or live-navigation screenshots.
The onboard PCF85063 and SD card are not used by the existing application.

See [setup, source references and runnable checks](WAVESHARE_143.md).
