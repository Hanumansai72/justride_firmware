# JustRide ESP32-S3 Firmware

Firmware for the JustRide motorcycle navigation device running on ESP32-S3 with BLE telemetry receiver and sunlight-readable display drivers.

## Project Structure
```
justride_firmware/
├── CMakeLists.txt              # Root CMake project configuration
├── sdkconfig.defaults          # Default ESP32-S3 and BLE configurations
├── main/
│   ├── CMakeLists.txt          # Main component registration
│   └── main.c                 # Firmware entry point (app_main)
└── README.md
```

## Building and Flashing (ESP-IDF)

### 1. Set the Target (ESP32-S3)
```bash
idf.py set-target esp32s3
```

### 2. Build the Firmware
```bash
idf.py build
```

### 3. Flash to Device and Monitor Output
```bash
idf.py -p COMx flash monitor
```
*(Replace `COMx` with your ESP32-S3 serial port, e.g., `COM3` or `COM4`)*

## Clock retention

The phone sends its full local date and time using Bluetooth Current Time Service.
The device stores a clock checkpoint in NVS on sync, BLE disconnect, software
restart, and each minute while running. Boot restores the saved date and time
before drawing the home screen. Until the first full sync, the date reads
`SYNC PHONE TIME` instead of January 1970. Legacy time-only packets preserve an
already known date.

To verify on hardware: install the updated firmware and app, connect once, check
the date/year/time, disconnect, remove power, then power on without the phone.
The home screen should resume from the saved clock and advance normally. Repeat
across midnight or a month/year boundary, then reconnect to confirm the phone
corrects the full calendar. Abrupt power loss can lose up to one minute since the
last checkpoint. Time spent fully powered off cannot be counted without a
battery-backed RTC.

## Touch wiring and swipe black-screen fix

The current display's **T_IRQ is connected to GPIO 14**. This GPIO must not be
treated as a physical power button. The power-button module, startup task,
short-press backlight toggle and long-press deep-sleep handling have been removed.
Touch coordinates and gestures continue to use SPI pressure polling; T_IRQ is
configured as an input. No separate power button is currently supported.

The reported serial log showed `BUTTONS: Short press` followed by
`Hardware Backlight set to 0%` immediately before `GESTURE: SWIPE LEFT`.
The LCD was rendering successfully while its backlight had been switched off.
Flash the rebuilt firmware onto the device to apply this fix; installing an APK
alone does not update the running firmware.


## OpenStreetMap side roads

The updated app sends OSM road geometry from the authenticated backend
`/api/maps/roads` endpoint. Update both firmware and app, and deploy the backend.
The device shows north-up roads, raw GPS position/heading and speed; Google
navigation stays in the phone. No Google route content is used on the OSM map.

Two 768-segment buffers use 13.5 KiB. Incoming geometry is validated by count,
sequence, coordinate bounds and CRC before committing. GPS updates pan the map;
heading/tilt changes redraw the marker region only. Attribution is always visible.
Road areas refresh near the coverage edge with at least 20 seconds between requests.
The app reports limited coverage when dense areas exceed 768 segments.

The prototype uses a public Overpass service, with bounded requests and six-hour
backend caching. See the app README for configuration and deployment details.
Native rendering/protocol checks and firmware compilation passed; real BLE
throughput, riding behavior and GPS coverage still require hardware verification.

## Music button touch alignment

The mounted panel has its raw X electrode direction reversed. Raw-X inversion
is applied once before display rotation (`TOUCH_INVERT_RAW_X=1` in `touch.h`),
so landscape taps and vertical swipes use the displayed coordinate system.
Reported PLAY/PAUSE taps at Y=70..76 now map to Y=169..163, inside the
118..178 playback row. Existing button bounds and Android media actions remain
unchanged. Music taps log their state/action mask, out-of-bounds hits and outgoing
commands to distinguish calibration from phone-session permission problems.
Flash the updated firmware; the running firmware is not changed by installing an APK.

## Route-only HUD geometry

The phone anchors near-route fixes to the projected road and fits route vertices
uniformly around the shared (160,160) rider origin. The native HUD maps that
origin to (207,148), uses a 300 m follow scale, and clips line segments instead
of clamping their endpoints. Changed map regions are composed in strips. Tilt
redraw restores the road under the marker rather than filling a blank rectangle.
Update both app and firmware for the complete correction. Google map background
configuration and the existing touch/music/sensor fixes remain as documented.

## Website-matched display palette

`main/display/display_theme.h` contains the website's CSS colors converted to
RGB565. The TFT color aliases and HUD share this palette, including dark
backgrounds, emerald accents and off-white text. Traffic signs and warnings
retain semantic colors. Rebuild/flash firmware to apply palette changes; no
APK change is required. RGB565 quantization and LCD characteristics affect
the exact physical color appearance.
