if(CONFIG_JUSTRIDE_ROUND_AMOLED)
    set(TOUCH_SOURCE "display/touch_ft3168.c" "board/waveshare143.c")
else()
    set(TOUCH_SOURCE "display/touch.c")
endif()
idf_component_register(SRCS "main.c"
                            "ota/ota.c"
                            "display/display.c"
                            "display/hud_ui.c"
                            "display/map_frame.c"
                            "display/display_tft.c"
                            "display/round_panel.c"
                            "display/round_ui.c"
                            ${TOUCH_SOURCE}
                            "ble/ble.c"
                            "time/rtc_time.c"
                            "board/device_power.c"
                            "mpu6500/mpu6500.c"
                       INCLUDE_DIRS "."
                                    "ota"
                                    "display"
                                    "ble"
                                    "time"
                                    "mpu6500"
                                    "board"
                       REQUIRES driver nvs_flash bt esp_adc app_update esp_timer)
