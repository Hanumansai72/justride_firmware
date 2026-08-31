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
