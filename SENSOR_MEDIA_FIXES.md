# Accelerometer, automatic calibration and media fixes

Updated 2026-10-19. Scope: sensor detection/calibration, app sensor controls and the firmware music controller. The unfinished navigation layout edits were restored before this work; map dimensions and navigation layout remain at the previously published version.

## Sensor corrections

- Sensor configuration writes and initial readings are checked. Invalid/zero data no longer creates a successful calibration baseline.
- Accelerometer tilt sampling runs even when the gyroscope is detected. A failed/unstable gyro calibration does not disable valid accelerometer operation.
- Calibration uses 40 consecutive valid, steady samples at the nominal 20 ms cadence. Each attempt is bounded to 120 reads. Sampling noise/movement resets the candidate window; failed reads are not averaged as zero.
- Calibration runs on the sensor task, not under the display/touch lock. Task creation errors and runtime sensor-read errors are reported. Settings → Motion / Calibrate shows status, roll, pitch and detected direction.
- Every inactive-to-active ride transition requests calibration automatically. Reroutes, paused/resumed telemetry and routine reconnect sync of an existing device ride do not request calibration again. A fresh device restoring an active ride calibrates on entering it.
- If ride-start calibration cannot obtain steady samples, it retries after five seconds between attempts while the ride remains active. Ride end/arrival/cancel cancels pending retries. Startup and optional manual calibration remain available; no manual action is required for the ride-start request.
- Keep the mounted device steady briefly at ride start. Accelerometer tilt is not an absolute GPS bearing or a speed measurement and does not replace phone navigation instructions.

## Companion app

Hardware Settings now contains Motion / Calibrate with actual device telemetry, live signed roll/pitch, movement and status. The optional Calibrate again button requests calibration over BLE and does not optimistically report success. Pending/automatic calibration disables duplicate requests. Stale telemetry hides angles and disables the button; disconnect clears the cached status. Older firmware shows a connection/update prompt instead of fabricated readings.

The existing FF32 capability response gains a third feature byte: `[1,2,1]`, with sensor support in bit zero. Existing navigation/map capability checks remain compatible. Sensor command on FF31 is `[0x1A,0]` for status and `[0x1A,1]` to request calibration. Firmware replies on FF32 with eight bytes: `[0x1A,1,state,motion,roll_i16_le,pitch_i16_le]`; angles are tenths of a degree. States: 0 unavailable, 1 needs calibration, 2 calibrating, 3 ready, 4 read error, 5 automatic retry waiting for stillness. Telemetry is emitted at up to five samples/second while BLE is connected; the existing two-second app timer refreshes missing telemetry, including an unavailable sensor whose sampling task could not start.

Relevant files: firmware `main/mpu6500/mpu6500.c/.h`, `main/display/display.c`, `main/ble/ble.c`; companion `sensor_status.dart`, `ble_navigation_service.dart`, `sensor_status_card.dart`, `device_settings_screen.dart`.

## Music corrections

- Firmware starts in waiting state instead of unsupported. Empty/missing Android bridge responses report bridge error; only non-Android platforms report unsupported.
- FF31 media states add 6 waiting and 7 bridge error. Actual permission, no-session, paused and playing states remain distinct. Playback controls remain disabled for error/waiting states.
- Music screen offers Retry media, or Enable on phone when notification access is missing. Only an explicit enable tap sends media command 8 and opens Android Notification Access settings. No permission is silently granted.
- Android permission detection checks the exact notification-listener component. Players exposing only the combined PLAY_PAUSE action are included in the action mask and targeted media-button dispatch; direct supported actions still use transport controls. No duplicate key event is sent after a direct action.
- Playback and metadata come from the selected actual Android MediaSession. Unsupported skip commands are not invented; lack of a player/session still requires opening a music app on the phone. Browser/iOS system media control is not implemented by this Android bridge.

Relevant companion files: `MainActivity.kt`, `system_media_bridge.dart`, `ble_media_packet.dart`, `ble_navigation_service.dart`.

## Verification

- ESP-IDF 5.5.5 firmware build/size gate passed for configured ESP32, 4 MB flash, PSRAM disabled. Final firmware binary: 656,256 bytes.
- Full Flutter suite: 129 tests passed. New tests exercise media bridge failures/permission/session states/commands, explicit settings launch, signed sensor packets, malformed packets, stale sensor display and compact calibration-card layout.
- Targeted analysis of new/changed bridge/model/widget/test files: no issues. The large existing hardware-settings source has pre-existing lint notes outside the inserted card. Whitespace gates passed.
- No sensor hardware, I2C electrical measurements, live phone/player control, physical calibration, BLE timing or device flashing was performed. Passing host tests/compilation does not establish those results. Confirm the actual sensor identity, SDA/SCL wiring, 3.3 V supply, NCS/AD0 configuration if Sensor not detected or Sensor read error persists.

## Install and check

Install the matching updated APK and firmware. In the app, open Hardware Settings → Motion / Calibrate. Start a ride while the mounted device is steady; watch Calibrating change to Ready automatically, then verify roll/pitch and direction while tilting. If movement prevents calibration, the app reports waiting for stillness and firmware retries automatically. End the ride and confirm retries stop.

For music, open the device Music screen. If Enable on phone appears, tap it and enable JustRide Media Control under Android Notification Access. Return to JustRide, open/play music in the desired player, and verify metadata plus play/pause/skip/volume on the physical device. Player-specific action support varies.

Source changes in this task have not been committed or pushed. Firmware remains on local `version-2.0`; companion remains on `main`.

## Final artifacts

- APK: `C:/Hanumansai_Projects/MobileApp-main/MobileApp-main/downloads/JustRide-auto-calibration-media-20261019.apk`, 59,709,866 bytes; SHA-256 `6cc4cea6921beaf8efa96f58ce39a03c0d11b2e6ca34b887cc616b7c58a242af`. Android release compilation passed.
- Firmware: `C:/Hanumansai_Projects/Justride/justride_firmware/build/justride_firmware.bin`, 656,256 bytes; SHA-256 `e1fb858529d5111a919164e694f35a68c1962f90fab5e66984b0d0d8dcb73978`.
- APK ZIP integrity passed, and its bundled firmware matches the final binary byte-for-byte.
- Logs: app `build/sensor-media-tests.log`, `build/sensor-media-analysis.log`, `build/sensor-media-apk-final.log`; firmware `build/sensor-media-build.log`.

## Follow-up: sensor continuity, swipe redraw, media buttons (2026-10-19)

The supplied boot log identifies firmware built at 00:33:13, before the latest sensor changes. MPU6500 WHO_AM_I 0x70 and real acceleration samples succeeded; the captured stationary samples do not demonstrate an I2C failure or a tilt response.

- Preserve the last accepted accelerometer baseline when a new calibration attempt is interrupted. Startup and ride-start calibration retry automatically after movement; waiting is reported separately from ready.
- Debounce finger release so a single noisy touch sample cannot split a swipe into accidental taps. Invalidate arrival/map/modal state on screen entry and force repaint on display power restoration.
- Draw each in-bounds opaque text glyph in one bulk transaction rather than 40 pixel-window operations. This reduces the black interval between clearing a screen and painting its labels, without allocating a full framebuffer.
- Queue Android media commands separately from BLE snapshot writes. Status polling cannot discard a button press or block the following phone command behind map traffic.
- Route remote-session volume to that session; local playback continues to use the music stream. If an advertised play/pause toggle rejects media key events, send the explicitly requested playback state through its transport controls. Unsupported player actions are dimmed on the device.
- Removed the sensor/media/map test files created during the earlier implementation. No new test or use-case files were added for this follow-up.

Verification: final ESP-IDF build passed, firmware binary 657,888 bytes, SHA-256 F3A1A7F00F96603EDED06C937340EF48B751B44CB95BCF98D0BB347E57F6CE85. Bundled app asset matches. Targeted Dart analysis and whitespace checks passed. No physical flash, tilt, swipe, or Android player verification was performed. The earlier artifact hashes above describe the previous build.

Follow-up APK release build passed: `C:/Hanumansai_Projects/MobileApp-main/MobileApp-main/downloads/JustRide-sensor-swipe-media-20261019.apk`, 59,710,158 bytes; SHA-256 `b944fee5335eaa30f0c960da08a5e8dc440a563717f98a0b86d96c8e11d4cae4`. APK archive integrity passed and its firmware asset matches the final firmware byte-for-byte. Logs: `build/sensor-swipe-media-build.log` (firmware) and companion `build/sensor-swipe-media-apk.log`. These changes remain local and have not been pushed.

## Follow-up: secondary maneuver removed and live motion (2026-10-19)

The remaining secondary maneuver renderer and both entry/update call sites were removed, including the "Then" status text and its cached fields. Its former card now displays speed. The map viewport and current maneuver remain compatible with the existing app protocol.

Motion delay had two code causes: the renderer forced a straight rider glyph and ignored the sensor value between navigation updates; recalibration collected up to 120 samples in a blocking loop. Calibration now collects one sample per 20 ms task tick while live sensing retains the accepted baseline. The local render loop reads continuous roll, applies a shorter 20 ms filter time constant, and repaints only the rider marker patch. A small independent map-marker background patch keeps rotation working while another map frame is received. No full-screen clear or phone navigation refresh is required for tilt updates. Hardware latency remains to be verified after flashing.

The production map screenshot's HTTP 503 was confirmed by the user to be an unconfigured server API key. Local configured LocationIQ download and shared quota reservation succeeded (HTTP 200). Production recheck is waiting for the user to configure LOCATIONIQ_API_KEY. Backend errors now distinguish missing setup, quota-store failure, provider failure, and timeouts without logging tokens. Client retry source honors Retry-After for 502/503/504 as well as 429.

No APK build was run and no test files were created for this follow-up. Only firmware compilation, Dart analysis, backend syntax checking, source inspection, and the configured map request were used for verification. Earlier APK links in this document refer to earlier work.

Final follow-up firmware build/size passed: 657,600 bytes; SHA-256 `99399636813E1D02441C2C898A6674777A211FFD6DE588708ABC478409E51EA1`. Firmware app asset matches. Targeted Dart analysis: no issues. Backend syntax and whitespace checks passed. Build log: `build/live-motion-no-secondary-build.log`. Physical tilt and screen response have not been measured.

## Follow-up: navigation display layout (2026-10-19)

Rearranged the existing navigation information for the 320x240 display: a centered medium maneuver card at (4,4), 114x114, with a 32px arrow, a bounded distance string and one instruction line; a dedicated 114x74 speedometer below at y=124 with 4x numeric typography; the existing 198x194 map remains the largest region. The footer uses fixed remaining-distance and time-left columns. Long values reduce font scale instead of crossing the map or neighboring column. Rerouting/wrong-turn messages use the same maneuver bounds. No secondary maneuver was reintroduced. No APK or test files were created for this layout update. The illustrative layout preview uses the firmware's bitmap font and UI coordinates; its sample roads are not a live map or hardware capture. Preview: `build/display-layout-preview.png`.

## Follow-up: swipe leads to black page (2026-10-19)

The user confirmed another swipe restores the display. Source inspection found that the photo page painted a full-screen image over its UI and did not redraw navigation controls on normal entry; a black image could therefore appear to be an off display. Incoming BLE image chunks also drew directly into the TFT without the UI lock, even after a swipe selected a different page. Image completion changed the active screen from the BLE callback without painting the whole page. These paths could clobber a newly selected page and leave it blank/mixed until the next swipe.

Image chunks now only write flash. Starting image storage uses the UI lock, and completion requests a photo redraw through the render task only if that page remains open. Photo title/navigation controls are painted after the image, and flash read failures show an error instead of silently leaving a blank area. Stored image headers validate length as well as magic. Abort/reset screen transitions use the locked screen API. Screen-completion logs include the screen ID and brightness for further hardware diagnosis.

The earlier release debounce incorrectly reused DEBOUNCE_TICKS=1. A separate three-sample release threshold now tolerates short read dropouts. Peak finger travel is tracked, and a short drag is no longer converted to a button tap; this prevents a noisy swipe from activating controls accidentally. No APK build or test files were added. Hardware swipe reproduction still requires flashing the updated firmware.

Swipe fix verification: ESP-IDF firmware build/size passed; bundled firmware asset matches SHA-256 65A242580A5ACC5CA32A5E72ABCF57AEC07D44497AF89A7231DE2400E2CC1473. Source whitespace check passed. No APK built, no physical swipe verification performed. Build log: build/swipe-black-screen-build.log.
