#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "JUSTRIDE";

void app_main(void)
{
    ESP_LOGI(TAG, "====================");
    ESP_LOGI(TAG, "JustRide Starting");
    ESP_LOGI(TAG, "====================");
    ESP_LOGI(TAG, "Version: 0.1.0");
    ESP_LOGI(TAG, "Device: ESP32-S3");
    ESP_LOGI(TAG, "Status: Ready");
    ESP_LOGI(TAG, "====================");
}
