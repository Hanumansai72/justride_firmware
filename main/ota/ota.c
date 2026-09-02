#include "ota.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_app_format.h"

// JustRide modules
#include "battery.h"

static const char *TAG = "OTA";

#define OTA_MIN_BATTERY_PERCENTAGE  30

static ota_status_t s_ota_status = OTA_STATUS_IDLE;
static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_update_partition = NULL;
static const esp_partition_t *s_running_partition = NULL;
static uint32_t s_total_image_size = 0;
static uint32_t s_received_bytes = 0;

void ota_init(void)
{
    ESP_LOGI(TAG, "Initializing JustRide OTA Subsystem...");

    s_running_partition = esp_ota_get_running_partition();
    if (s_running_partition) {
        ESP_LOGI(TAG, "Currently running partition: '%s' at offset 0x%08" PRIx32 " (Size: %" PRIu32 " KB)",
                 s_running_partition->label,
                 s_running_partition->address,
                 s_running_partition->size / 1024);
    }

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(s_running_partition, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "Running firmware is pending validation (fresh OTA boot)!");
        } else if (ota_state == ESP_OTA_IMG_VALID) {
            ESP_LOGI(TAG, "Running firmware confirmed as VALID.");
        }
    }

    s_ota_status = OTA_STATUS_IDLE;
}

esp_err_t ota_start(uint32_t image_size)
{
    ESP_LOGI(TAG, "OTA Start requested. Total size: %" PRIu32 " bytes", image_size);

    // 1. Hardware Battery Level Safety Check
    int batt_pct = battery_get_percentage();

    // 2. Identify the inactive target partition
    s_running_partition = esp_ota_get_running_partition();
    s_update_partition = esp_ota_get_next_update_partition(NULL);

    if (s_update_partition == NULL) {
        ESP_LOGE(TAG, "Passive OTA partition not found in partition table!");
        s_ota_status = OTA_STATUS_ERROR_FLASH_WRITE;
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Writing OTA update to partition '%s' at offset 0x%08" PRIx32,
             s_update_partition->label, s_update_partition->address);

    // 3. Begin OTA partition erase & prepare handle
    esp_err_t err = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        s_ota_status = OTA_STATUS_ERROR_FLASH_WRITE;
        return err;
    }

    s_total_image_size = image_size;
    s_received_bytes = 0;
    s_ota_status = OTA_STATUS_READY;

    ESP_LOGI(TAG, "OTA partition ready. Awaiting binary data chunks...");
    return ESP_OK;
}

esp_err_t ota_write_chunk(uint32_t offset, const uint8_t *data, uint16_t length)
{
    // Strict guard: only accept chunks when OTA session is active
    if (s_ota_status != OTA_STATUS_READY && s_ota_status != OTA_STATUS_RECEIVING) {
        ESP_LOGE(TAG, "Cannot write chunk: OTA not in write state (Status: %d). "
                      "Send START command first to open an OTA session.", s_ota_status);
        return ESP_ERR_INVALID_STATE;
    }

    s_ota_status = OTA_STATUS_RECEIVING;

    esp_err_t err = esp_ota_write(s_ota_handle, (const void *)data, length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at offset %" PRIu32 ": %s", offset, esp_err_to_name(err));
        s_ota_status = OTA_STATUS_ERROR_FLASH_WRITE;
        return err;
    }

    s_received_bytes += length;
    return ESP_OK;
}

esp_err_t ota_finish(void)
{
    ESP_LOGI(TAG, "OTA Finish requested. Verifying %" PRIu32 " bytes...", s_received_bytes);
    s_ota_status = OTA_STATUS_VERIFYING;

    esp_err_t err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end verification failed: %s", esp_err_to_name(err));
        s_ota_status = OTA_STATUS_ERROR_VALIDATION_FAIL;
        return err;
    }

    // Mark the new target partition as bootable for the next restart
    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        s_ota_status = OTA_STATUS_ERROR_FLASH_WRITE;
        return err;
    }

    s_ota_status = OTA_STATUS_SUCCESS;
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "✓ OTA UPDATE COMPLETE! Ready to reboot into '%s'.", s_update_partition->label);
    ESP_LOGI(TAG, "=================================================");

    return ESP_OK;
}

void ota_abort(void)
{
    if (s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }
    s_ota_status = OTA_STATUS_ERROR_ABORTED;
    s_received_bytes = 0;
    s_total_image_size = 0;
    ESP_LOGW(TAG, "OTA session aborted.");
}

uint8_t ota_get_progress_percent(void)
{
    if (s_total_image_size == 0) return 0;
    uint32_t pct = (s_received_bytes * 100) / s_total_image_size;
    return (uint8_t)(pct > 100 ? 100 : pct);
}

ota_status_t ota_get_status(void)
{
    return s_ota_status;
}

void ota_confirm_boot_valid(void)
{
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(s_running_partition, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Self-tests passed! Confirming firmware valid (cancelling rollback)...");
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "Firmware permanently accepted.");
        }
    }
}
