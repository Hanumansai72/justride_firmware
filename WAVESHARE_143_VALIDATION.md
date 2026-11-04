# Verification — 4 November 2026

| Check | Result |
|---|---|
| ESP-IDF 5.5.5 / Waveshare signed-update profile | Pass; unsigned/padded application 983,040 bytes; signed image 987,136 bytes of the unchanged 1,769,472-byte OTA slot (about 44% free) |
| Original ESP32 2.8-inch profile | Pass; retained 4 MB target compiles independently |
| Signature tooling | Native ESP-IDF verification accepts the signed image, rejects wrong-key, modified and truncated images |
| Firmware host checks | Pass: actual QSPI/FT3168 mocked I/O, power/motion, steady guidance and turn-direction checks, warning completion/stale suppression, day/night boundaries and larger call hit regions |
| Touch/wake driver suite | Pass: 101 checks × four rotations, 404 total |
| Map receiver/camera bounds | Pass |
| Round renderer | Production C captures; identical repeated guidance frames; turn direction still updates; no chevrons or timed blink |
| Focused companion-app regression set | 51 tests passed, including multi-stop edits/skip, failed next-leg retention, confirmed intermediate arrival, late responses, trail persistence, power guards and navigation/BLE/clock/zoom checks |
| Changed app code analysis | No errors or warnings; informational style lints remain |

The included private companion-app source remains in its separate checkout and is
not published in this public firmware repository. Release packaging verifies that
its S3 asset matches the signed USB application, while its separate original ESP32
asset stays intact. The APK retains the project's prototype/debug signing key.

The broader app suite is not clean: speed_filter_test.dart references
missing SpeedFilter.toKmh, and ui_cleanup_test.dart depends on unsupported desktop
Bluetooth plus outdated settling/image expectations. Those unrelated tests were
left intact. The expanded run also has three history/layout failures: missing
desktop path-provider setup in the upload test, registration assertion failure,
and dashboard API/layout expectations. These are not covered by the passing
focused set and remain unresolved. No CodeRabbit review is claimed.

## Memory and performance

The padded unsigned application is 983,040 bytes; signing adds one 4,096-byte
sector. Reference-style fonts add flash data, without another framebuffer.
ELF reports .data 40,790 bytes and .bss 10,160 bytes;
these section values are not runtime free heap. Actual runtime memory/latency
before/after was not measured because no board is attached.

No framebuffer or FreeRTOS task was added. Existing PSRAM shadow and DMA buffers
are reused. Native signature verification uses the existing main task with its
stack raised from 3,584 to 8,192 bytes, plus a fixed four-entry OTA queue (~2.1 KB)
and small bounded destination/settings state. Existing 30-second render/heap/stack
serial metrics remain available for a physical benchmark.

## Physical checks still required

No device was flashed or bench-tested. Actual panel/touch orientation, gloves,
mounting/gyro behavior, power draw/ADC calibration, background Android GPS, BLE
pairing/reconnection, interrupted OTA and rollback remain unverified on hardware.
Host signature rejection does not prove hardware power-loss recovery.
The stock board still cannot report charging/full status. PCF85063 and SD remain
unused. Preview images use sample telemetry, not live navigation or photographs.

See [feature behavior, provider constraints and signed installation](RIDER_FEATURES.md)
and [board setup](WAVESHARE_143.md).
