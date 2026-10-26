JustRide OSM side-road prototype

Install the updated app, flash this firmware and deploy the updated Backend before riding.
Device: north-up OpenStreetMap roads, raw GPS marker, heading and speed. Phone: Google navigation.
No map API key or Google billing is needed for this device map. Backend Redis is required in production.
Public Overpass service availability and OSM road coverage vary. Dense areas retain the nearest 768 segments.
The old GPIO 14 touch/power-button fix is included. Do not attach a power button to the touch IRQ pin.

In an ESP-IDF terminal in this folder (replace COMx):
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Checks passed: live OSM fetch, backend validation/quota checks, Dart protocol check, native CRC/sequence/panning/partial redraw capture, targeted Dart analysis and ESP-IDF build.
Real phone/device ride validation is still required; no hardware was flashed during these checks.
