#include "rtc_time.h"

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"

static const char *TAG = "TIME_RTC";

static bool s_time_synced = false;

void time_set_current_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    if (hours > 23) hours = 23;
    if (minutes > 59) minutes = 59;
    if (seconds > 59) seconds = 59;

    struct tm tm_target = {0};
    tm_target.tm_year = 2026 - 1900;
    tm_target.tm_mon = 8;
    tm_target.tm_mday = 2;
    tm_target.tm_hour = hours;
    tm_target.tm_min = minutes;
    tm_target.tm_sec = seconds;

    time_t t = mktime(&tm_target);
    struct timeval tv = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&tv, NULL);

    s_time_synced = true;
    ESP_LOGI(TAG, "RTC synchronized from phone: %02d:%02d:%02d", hours, minutes, seconds);

    // Immediately push live time to display
    display_set_time(hours, minutes);
}

void time_set_epoch(uint32_t epoch_seconds)
{
    struct timeval tv = {.tv_sec = (time_t)epoch_seconds, .tv_usec = 0};
    settimeofday(&tv, NULL);

    s_time_synced = true;
    time_t now = (time_t)epoch_seconds;
    struct tm tm_info;
    localtime_r(&now, &tm_info);

    ESP_LOGI(TAG, "RTC synchronized via Unix timestamp: %02d:%02d:%02d",
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);

    // Immediately push live time to display
    display_set_time(tm_info.tm_hour, tm_info.tm_min);
}

bool time_is_synchronized(void)
{
    return s_time_synced;
}

// Background FreeRTOS task: runs once every second.
// Whenever the minute advances, it updates the display live.
static void rtc_clock_task(void *pvParameters)
{
    int last_minute = -1;

    while (1) {
        time_t now;
        time(&now);
        struct tm tm_info;
        localtime_r(&now, &tm_info);

        if (tm_info.tm_min != last_minute) {
            last_minute = tm_info.tm_min;
            display_set_time(tm_info.tm_hour, tm_info.tm_min);
            ESP_LOGI(TAG, "Live clock tick -> %02d:%02d", tm_info.tm_hour, tm_info.tm_min);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t time_init(void)
{
    ESP_LOGI(TAG, "Initializing Real-Time Clock engine...");

    BaseType_t ret = xTaskCreate(rtc_clock_task, "rtc_clock_task", 2048, NULL, 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create rtc_clock_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTC background task started.");
    return ESP_OK;
}
