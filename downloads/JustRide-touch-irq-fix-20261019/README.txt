JustRide GPIO 14 / touch IRQ fix

GPIO 14 is display T_IRQ. The physical power-button module and its backlight/deep-sleep handlers have been removed.

Firmware build and partition size checks passed. Compiled ELF and binary contain no button task, button init or display-power-toggle implementation. Real hardware swipe verification remains pending.

From an ESP-IDF terminal in this folder, replace COMx with the device port:
python -m esptool --chip esp32 -p COMx -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Alternatively build/flash from the project with idf.py -p COMx flash monitor.
Installing an APK alone does not replace the running device firmware.
