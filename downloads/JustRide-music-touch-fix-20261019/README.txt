JustRide music touch alignment fix

The panel raw X axis is inverted before display rotation. Reported PLAY/PAUSE
coordinates (134..138,70..76) now become (134..138,163..169), inside its button.
This also changes vertical swipe direction to match the mounted panel.
Existing media button bounds, commands and Android action permissions are unchanged.

Flash from an ESP-IDF terminal in this extracted folder, replacing COMx:
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

No new APK is needed for this firmware-only correction. The current app can stay installed.
Startup must report: Touch mapping: rotation=1 raw-X-inverted=1
Tap PLAY/PAUSE and VOL + on the music screen. Look for:
MEDIA TAP x=... y=... connected=1 state=... actions=...
MEDIA command -> phone: 1 (play), 2 (pause), 3 (next), 4 (previous), 5 (volume up), 6 (volume down)
A grey action can be unavailable in the phone's active music session.
If no command is sent, the new logs distinguish button bounds and unavailable actions.

ESP-IDF build and native checks of the production coordinate mapping passed.
Physical panel alignment and actual phone playback still need verification after flashing.
No temporary test source files were retained.
