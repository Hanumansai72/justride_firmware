if(CONFIG_JUSTRIDE_ROUND_AMOLED)
    set(TOUCH_SOURCE "display/touch_cst9217.c")
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
                            "mpu6500/mpu6500.c"
                       INCLUDE_DIRS "."
                                    "ota"
                                    "display"
                                    "ble"
                                    "time"
                                    "mpu6500"
                       REQUIRES driver nvs_flash bt esp_adc app_update esp_timer)
