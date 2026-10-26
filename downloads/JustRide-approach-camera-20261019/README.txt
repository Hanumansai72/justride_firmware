JustRide current-approach camera

Install the matching approach-camera APK and flash this firmware.
At 131 m to a right turn, the current road fills the forward map area.
The current bend is revealed within 60 m, with a short exit capped before the
following maneuver. Real route curves are preserved. Geometry updates after
3 m of GPS movement and when the current maneuver changes.
The APK retains the media recovery fix and uses the route-focused camera by default.
The speed/trip/current-turn panels remain available.

In an ESP-IDF terminal in this extracted folder, replace COMx:
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Checks passed: Dart analysis, approach geometry at 131/100/60/30 m, native
render capture and ESP-IDF compilation. Included PNGs are renderer checks
with a sample route, not live hardware captures. Physical GPS/BLE verification
is still required. Temporary test/probe sources were removed.
