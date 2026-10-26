JustRide route geometry/display fix

Install the matching route-display-fix APK and flash this firmware.
The app anchors the route to the projected road for GPS fixes within 25 m.
Route points are fitted uniformly, preserving turn geometry instead of forcing
vertices against the display boundary. Firmware uses a consistent follow scale
and clips line segments. The rider marker restores the road underneath it.
Changed map regions are composed in strips, without a preliminary screen clear.
Music/touch alignment and sensor fixes are retained. No OSM integration is included.

From an ESP-IDF terminal in this extracted folder, replacing COMx:
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Checks passed: ESP-IDF build, targeted Dart analysis, route-origin/left-turn/hairpin
checks, native renderer capture and marker-region redraw/clipping checks.
Temporary probe/test source files were removed.
Physical device GPS/BLE ride verification is still required.
The route-only HUD does not draw background side streets; Google Static Maps
background imagery still requires the configured backend and Google billing.
