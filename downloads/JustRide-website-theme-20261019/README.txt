JustRide website color theme for the device

Palette source: justride-customer/src/index.css :root
Background #0b100d, panel #141a16, alternate panel #1a211c,
emerald #34d399, dark emerald #087c59, text #f5f7f5, muted #aab5ad.
The device converts these to RGB565. Warning/traffic-sign colors remain distinct.
Shared colors cover navigation, home, music, settings, brightness and boot UI.
The route line and its halo now use emerald rather than cyan/blue.

Flash this firmware. Your currently installed approach-camera/media-recovery APK
can stay installed; no APK update is required to apply the display colors.

In an ESP-IDF terminal in this extracted folder, replace COMx:
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

ESP-IDF build and native preview rendering passed. The PNG is a native renderer
preview using sample navigation data, not a photo of the physical display.
Final color appearance depends on the LCD; verify after flashing.
