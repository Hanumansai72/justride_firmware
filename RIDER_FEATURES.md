# Rider features — Waveshare 1.43-inch

This release targets the existing ESP32-S3-Touch-AMOLED-1.43, 466×466 panel,
16 MB flash and 8 MB PSRAM. It does not change the BOM. The original 2.8-inch
profile still compiles; the separate version-2.0 working checkout is untouched.

## Delivered behavior

| Feature | Implementation and limits |
|---|---|
| Turn blink | Medium green chevrons on the left at **15 metres or closer**, 500 ms on/off. Map and white pointer remain visible. Suppressed after completion or during stale/GPS/recovery/paused/overview states. Optional phone vibration retains its separate earlier 100 m warning. |
| Multi-stop | Phone navigation → **Stops**: search/add, remove or move future stops; active stop stays first. **Skip active stop** changes it explicitly. Accurate, slow, confirmed arrival advances once. Only the active leg is requested; future-stop edits are local. Failed next-leg requests retain the new destination and suppress the old guidance until retry succeeds. Device shows the active stop and following maneuver. Up to 20 remaining stops. |
| Larger controls | Device settings toggle enlarges round call/music controls and their touch regions, preserving gaps and normal mode. This does not change FT3168 capacitance sensitivity. Ordinary-glove detection is unverified. |
| Hazard alerts | Compact sharp-left/right warning only from provider-supplied sharp maneuver codes. Continuous indication for the current maneuver avoids repeated popups; it clears after passage/completion, reroute or stale guidance. Threshold is six seconds of travel, clamped to 50–200 m. No invented closures, dangerous junctions or hazard database. |
| Journey recording | Existing phone GPS stream only. Fresh fixes ≤25 m accuracy; sampled trail every five seconds, app-private JSONL writes batched about every 30 seconds. Outages >10 seconds, rejected accuracy and pause/resume split the trail. Distance never bridges an outage. Average speed is distance / active elapsed time; pauses are excluded. Phone history remains available offline with a simple GPS-trail view and summary. No basemap is implied. Owner-scoped checkpoints preserve totals and stops; a crash may lose the latest 30 seconds. |
| Offline continuity | Current route and instructions remain in memory during internet loss; GPS/BLE continue. Off-route requests back off and recover when online. Stops, journey totals and the actual GPS trail survive a restart. **Restoring Google route geometry after app termination still needs an online request**; provider route data and map tiles are not permanently cached. No offline rerouting engine is claimed. |
| Brightness | Optional phone-clock day/night schedule on the device: 85% at 07:00–18:59, 35% otherwise. Brightness changes only when the target changes. Manual display/app changes disable automatic mode. Preferences persist locally and resync on connection. No board light sensor exists; phone light readings are not used as a proxy for the mounted display. |
| OTA | Running version and signed-update capability exposed over existing BLE. HTTPS direct downloads only, bounded by the existing OTA partition. App/device reject active rides and stale/unknown/low power; valid inferred USB power or battery estimate ≥30% required. Native ESP-IDF signature verification gates activation. Flash operations use app_main's existing loop and a four-entry static queue. Disconnects invalidate queued work; app waits for READY and written-to-flash status. Incomplete/invalid images never become the boot partition. Rollback acceptance requires display-task heartbeat and synchronized BLE startup. |

## Signed initial installation and future builds

The supplied USB package contains a **signed** application. Install this trusted
bootstrap by USB before using OTA. A software-verification build flashed without
its signature cannot run properly. Do not flash the unsigned build output.

From an ESP-IDF 5.5.5 environment, a fresh signed-profile build is:

```powershell
idf.py -B build-signed -DSDKCONFIG=sdkconfig.release '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.signed' build
python tools/sign_release.py --build build-signed --key C:/private/your-existing-rsa3072.pem
python tools/test_signed_release.py --image build-signed/signed/justride_firmware.bin --key C:/private/your-existing-rsa3072.pem
```

Use `build-signed/signed/justride_firmware.bin` in the USB package and APK, along
with that build's bootloader/partition table/initial OTA data. Keep and securely
back up the same private key outside all repositories; changing it breaks the
software chain of trust for installed units. The local release key is stored
outside both checkouts under the user's `.codex/justride-signing` directory.
No private key is embedded in firmware, APK, ZIP, previews or Git.

This profile uses RSA-3072 signed-app verification without hardware secure boot.
It deliberately does **not** burn eFuses, enable flash encryption or block physical
reflashing. It does not add bonded/MITM-protected BLE or anti-downgrade eFuses.
Signed images establish image authenticity, not authorization of a particular
phone. Those are separate production provisioning decisions. The APK retains
the project's prototype/debug release signing configuration.

## Provider and hardware limits

Google's standard Routes policy restricts caching and has attribution/device-use
requirements: [official policy](https://developers.google.com/maps/documentation/routes/policies).
Persistent provider-route restart recovery remains conditional on an appropriate
license or approved provider. Actual recorded GPS coordinates are separate from
provider route content and are saved locally on the phone.

Native signed-update details and initial trust requirements:
[ESP-IDF signed-app verification](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32s3/security/secure-boot-v2.html#signed-app-verification-without-hardware-secure-boot).

See NAVIGATION_GAPS.md for existing U-turn-side, flyover, roundabout-exit and
parallel/stacked-road limitations. Charging/full status remains unknown on the
stock board; VCC is not battery voltage while USB-powered.

## Validation and memory

49 focused Flutter tests passed, including six actual NavigationService tests,
arrival progression, skip/failure handling, local trail round-trip, power guards,
navigation accuracy, BLE packets, touch zoom, clock and existing gap diagnostics.
The broader pre-existing app test failures remain documented separately.
Native host checks cover panel/touch/power, 15/16 m blink boundary, warning bounds,
day/night boundaries, enlarged call hit regions, wake behavior and rendering.
Native signature tooling accepts the signed release and rejects wrong-key,
modified and truncated images. These do not replace a physical interrupted-OTA test.

No new FreeRTOS task or framebuffer was added. The signed profile increases main
task stack from 3,584 to 8,192 bytes for verification. The fixed OTA queue holds
four ~524-byte entries; destination state/queue and rider settings are small,
bounded additions. Existing panel shadow (434,312 bytes) and 16-row DMA transfer
buffer (14,912 bytes) are reused. Existing 30-second render/heap/stack metrics
remain available on serial. Runtime heap, draw latency and battery savings have
not been measured on physical hardware.

Software previews use the production C renderer and sample telemetry. Bench-test
panel wake/brightness, both touch modes with actual gloves, phone background GPS,
radio reconnection, ADC thresholds, power-loss OTA and automatic rollback before
calling this a production release.

## Actual source changes

Firmware: ride_ux/round_ui/display for turn warnings, lookahead, rider options and
schedule; ble/ota/main/device_power for version, queued OTA, signature/power gates
and startup health; signed defaults, signing checks and preview/test fixtures.
Companion app: existing navigation service/screen, navigation session, ride history
service/screens, BLE navigation service, OTA service and device settings. No second
GPS/BLE/database service was introduced. `path_provider` was already installed
transitively and is now declared directly; unused `google_fonts` and
`smooth_page_indicator` were removed after checking for references.
