#include "buttons.h"
#include "display.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTONS";

#define POLL_INTERVAL_MS      20
#define SHORT_PRESS_MIN_MS    50    // Min threshold to filter out contact bounce
#define LONG_PRESS_THRESH_MS  2000  // 2.0 seconds hold for deep sleep / power off

static TaskHandle_t s_button_task_handle = NULL;

bool buttons_is_display_on(void)
{
    return display_get_power();
}

void buttons_toggle_display_power(void)
{
    bool current = display_get_power();
    display_set_power(!current);
}

static void button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Power button task started on GPIO %d (Active LOW)", BUTTON_PIN_POWER);

    uint32_t press_duration_ms = 0;
    bool long_press_handled = false;

    while (1) {
        int level = gpio_get_level((gpio_num_t)BUTTON_PIN_POWER);

        if (level == 0) {
            // Button is pressed (Active LOW, pulled to GND)
            press_duration_ms += POLL_INTERVAL_MS;

            if (press_duration_ms >= LONG_PRESS_THRESH_MS && !long_press_handled) {
                // Long press detected -> Power Off (Deep Sleep)
                long_press_handled = true;
                ESP_LOGW(TAG, "Long press (%u ms) detected! Powering down into Deep Sleep...", (unsigned)press_duration_ms);

                // Turn off display backlight immediately
                display_set_power(false);

                // Wait for the user to release the button before going to sleep so it doesn't immediately wake
                while (gpio_get_level((gpio_num_t)BUTTON_PIN_POWER) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // Debounce release

                ESP_LOGI(TAG, "Configuring wake-up on GPIO %d (ext0, LOW) and entering Deep Sleep...", BUTTON_PIN_POWER);
                esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN_POWER, 0);
                esp_deep_sleep_start();
            }
        } else {
            // Button is released (level == 1)
            if (press_duration_ms >= SHORT_PRESS_MIN_MS && !long_press_handled) {
                // Valid short press detected -> Toggle display on/off
                ESP_LOGI(TAG, "Short press (%u ms) detected: Toggling display power", (unsigned)press_duration_ms);
                buttons_toggle_display_power();
            }

            // Reset trackers
            press_duration_ms = 0;
            long_press_handled = false;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t buttons_init(void)
{
    ESP_LOGI(TAG, "Initializing Power Button on GPIO %d (Internal Pull-Up)...", BUTTON_PIN_POWER);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN_POWER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", BUTTON_PIN_POWER, esp_err_to_name(err));
        return err;
    }

    BaseType_t ret = xTaskCreate(button_task, "button_task", 2048, NULL, 4, &s_button_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Power Button initialized successfully (Short press: Screen On/Off, Long press: Deep Sleep)");
    return ESP_OK;
}
