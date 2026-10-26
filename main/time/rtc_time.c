#include "rtc_time.h"

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "display.h"

static const char *TAG = "TIME_RTC";

static bool s_time_synced = false;
static SemaphoreHandle_t s_save_mutex;
static int64_t s_last_saved = 0;

// Reject the uninitialized Unix clock and corrupt flash values.
static bool valid_epoch(int64_t value)
{
    return value >= 1577836800LL && value <= 4102444799LL; // 2020 through 2099
}

void time_save_checkpoint(void)
{
    if (!s_save_mutex) return;
    xSemaphoreTake(s_save_mutex, portMAX_DELAY);
    time_t now = time(NULL);
    if (s_time_synced && valid_epoch(now) && now != s_last_saved) {
        nvs_handle_t handle;
        esp_err_t err = nvs_open("clock", NVS_READWRITE, &handle);
        if (err == ESP_OK) {
            err = nvs_set_i64(handle, "epoch", (int64_t)now);
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }
        if (err == ESP_OK) s_last_saved = now;
        else ESP_LOGW(TAG, "Clock checkpoint failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_save_mutex);
}

void time_set_current_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    if (hours > 23) hours = 23;
    if (minutes > 59) minutes = 59;
    if (seconds > 59) seconds = 59;

    // Legacy time-only packets must preserve the restored calendar date.
    // A device that has never received a date waits for a full phone sync.
    if (!s_time_synced) return;
    time_t now = time(NULL);
    struct tm tm_target;
    localtime_r(&now, &tm_target);
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
    time_save_checkpoint();
}

bool time_set_datetime(uint16_t year, uint8_t month, uint8_t day,
                       uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    if (year < 2020 || year > 2099 || month < 1 || month > 12 ||
        day < 1 || day > 31 || hours > 23 || minutes > 59 || seconds > 59)
        return false;
    struct tm value = {.tm_year = year - 1900, .tm_mon = month - 1,
        .tm_mday = day, .tm_hour = hours, .tm_min = minutes,
        .tm_sec = seconds, .tm_isdst = -1};
    time_t epoch = mktime(&value);
    // mktime normalizes impossible dates, such as February 31.
    if (!valid_epoch(epoch) || value.tm_mon != month - 1 || value.tm_mday != day)
        return false;
    time_set_epoch((uint32_t)epoch);
    return true;
}

void time_set_epoch(uint32_t epoch_seconds)
{
    if (!valid_epoch(epoch_seconds)) return;
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
    time_save_checkpoint();
}

bool time_is_synchronized(void)
{
    return s_time_synced;
}

void time_get_date_str(char *buf, size_t buf_len)
{
    if (!buf || buf_len < 16) return;
    if (!s_time_synced) {
        snprintf(buf, buf_len, "SYNC PHONE TIME");
        return;
    }
    time_t now;
    time(&now);
    struct tm tm_info;
    localtime_r(&now, &tm_info);

    static const char *const s_wday_names[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
    static const char *const s_mon_names[]  = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                               "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

    int wday = (tm_info.tm_wday >= 0 && tm_info.tm_wday <= 6) ? tm_info.tm_wday : 3;
    int mon  = (tm_info.tm_mon >= 0 && tm_info.tm_mon <= 11) ? tm_info.tm_mon : 8;
    int year = (tm_info.tm_year > 0) ? (tm_info.tm_year + 1900) : 2026;
    int mday = (tm_info.tm_mday >= 1 && tm_info.tm_mday <= 31) ? tm_info.tm_mday : 9;

    snprintf(buf, buf_len, "%s, %02d %s %04d", s_wday_names[wday], mday, s_mon_names[mon], year);
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

        if (s_time_synced && tm_info.tm_min != last_minute) {
            last_minute = tm_info.tm_min;
            display_set_time(tm_info.tm_hour, tm_info.tm_min);
            ESP_LOGI(TAG, "Live clock tick -> %02d:%02d", tm_info.tm_hour, tm_info.tm_min);
            // At most one flash checkpoint per minute, including offline use.
            time_save_checkpoint();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t time_init(void)
{
    ESP_LOGI(TAG, "Initializing Real-Time Clock engine...");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    s_save_mutex = xSemaphoreCreateMutex();
    if (!s_save_mutex) return ESP_ERR_NO_MEM;

    nvs_handle_t handle;
    int64_t saved = 0;
    if (nvs_open("clock", NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_i64(handle, "epoch", &saved) != ESP_OK) saved = 0;
        nvs_close(handle);
    }
    time_t now = time(NULL);
    if (!valid_epoch(now) && valid_epoch(saved)) {
        struct timeval tv = {.tv_sec = (time_t)saved, .tv_usec = 0};
        settimeofday(&tv, NULL);
        now = (time_t)saved;
    }
    s_time_synced = valid_epoch(now);
    s_last_saved = saved;
    struct tm restored;
    localtime_r(&now, &restored);
    display_set_time(s_time_synced ? restored.tm_hour : 0,
                     s_time_synced ? restored.tm_min : 0);
    ESP_ERROR_CHECK(esp_register_shutdown_handler(time_save_checkpoint));

    BaseType_t ret = xTaskCreate(rtc_clock_task, "rtc_clock_task", 4096, NULL, 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create rtc_clock_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTC background task started.");
    return ESP_OK;
}
