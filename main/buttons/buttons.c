#include "buttons.h"
#include "esp_log.h"

static const char *TAG = "BUTTONS";

esp_err_t buttons_init(void)
{
    ESP_LOGI(TAG, "Initializing buttons GPIO / interrupts...");
    // TODO: Configure button GPIOs and ISRs
    return ESP_OK;
}
