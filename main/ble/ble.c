#include "ble.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// ESP-IDF NimBLE stack headers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Standard C library time headers
#include <time.h>
#include <sys/time.h>

// JustRide modules
#include "rtc_time.h"
#include "ota.h"
#include "display.h"
#include "mpu6500.h"

static const char *TAG = "BLE";

// The advertised BLE device name for the motorcycle unit
#define BLE_DEVICE_NAME "JUSTRIDE"

// Bluetooth SIG Standard Current Time Service (CTS)
#define BLE_SVC_CURRENT_TIME_UUID16      0x1805
#define BLE_CHR_CURRENT_TIME_UUID16      0x2A2B

// JustRide Custom OTA Service UUIDs (16-bit aliases for clean GATT table)
#define BLE_SVC_OTA_UUID16               0xFF10
#define BLE_CHR_OTA_CONTROL_UUID16       0xFF11
#define BLE_CHR_OTA_DATA_UUID16          0xFF12
#define BLE_CHR_OTA_STATUS_UUID16        0xFF13

// JustRide Image Transfer Service UUIDs
#define BLE_SVC_IMG_UUID16               0xFF20
#define BLE_CHR_IMG_CONTROL_UUID16       0xFF21  // Write: START/FINISH/ABORT/RESET
#define BLE_CHR_IMG_DATA_UUID16          0xFF22  // Write no-rsp: [seq_lo][seq_hi][...rgb565...]
#define BLE_CHR_IMG_STATUS_UUID16        0xFF23  // Notify: [status][progress%]

// JustRide Navigation Service UUIDs (0xFF30)
#define BLE_SVC_NAV_UUID16               0xFF30
#define BLE_CHR_NAV_DATA_UUID16          0xFF31  // Write: binary / JSON navigation packet
#define BLE_CHR_NAV_STATUS_UUID16        0xFF32  // Notify / Read: device status


// Navigation Opcodes matching Flutter BleNavOpCode
#define NAV_OP_START                     0x01
#define NAV_OP_UPDATE                    0x02
#define NAV_OP_REROUTE                   0x03
#define NAV_OP_PAUSE                     0x04
#define NAV_OP_RESUME                    0x05
#define NAV_OP_CANCEL                    0x06
#define NAV_OP_ARRIVED                   0x07
#define NAV_OP_STOP                      0x08
#define NAV_OP_STATUS                    0x09
#define NAV_OP_SYNC                      0x0A
#define NAV_OP_ROUTE_SEGMENT             0x0B
#define NAV_OP_SET_BRIGHTNESS            0x0C
#define NAV_OP_RESTART                   0x0D
#define NAV_OP_RESET                     0x0E
#define NAV_OP_NOTIFICATION              0x0F
#define NAV_OP_DIAGNOSTICS               0x10
#define NAV_OP_CONTEXT                   0x11

// Image Control OpCodes
#define IMG_CMD_START                    0x01
#define IMG_CMD_FINISH                   0x02
#define IMG_CMD_ABORT                    0x03
#define IMG_CMD_RESET                    0x04

// Image Status Codes
#define IMG_STATUS_READY                 0x10
#define IMG_STATUS_RECEIVING             0x11
#define IMG_STATUS_SUCCESS               0x12
#define IMG_STATUS_ERROR_SIZE            0x13
#define IMG_STATUS_ERROR_ABORTED         0x14

// OTA Control OpCodes
#define OTA_CMD_START                    0x01
#define OTA_CMD_FINISH                   0x02
#define OTA_CMD_ABORT                    0x03
#define OTA_CMD_REBOOT                   0x04

// Connection state tracking
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_ota_status_val_handle = 0;
static uint16_t s_img_status_val_handle = 0;
static uint16_t s_nav_status_val_handle = 0;
static bool s_is_connected = false;
static uint8_t s_ota_last_reported_pct = 0;
static uint16_t s_ota_expected_seq = 0;
static uint8_t s_own_addr_type;
static TimerHandle_t s_adv_timer = NULL;

// Image transfer state
#define IMG_TOTAL_BYTES  (320u * 240u * 2u)   // 153 600 bytes (320x240 RGB565)
#define IMG_CHUNK_SIZE   128u
static uint32_t s_img_bytes_received = 0;
static bool     s_img_transfer_active = false;
static uint8_t s_img_status = IMG_STATUS_READY;


// Forward declarations
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int ble_ota_control_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_ota_data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_ota_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_control_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static uint32_t s_route_revision;
static bool s_revision_received;
static uint16_t s_img_expected_seq;

static int ble_nav_data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_nav_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);


// =============================================================================
// PUBLIC GETTERS & RESET CONTROLS
// =============================================================================

bool ble_is_connected(void)
{
    return s_is_connected;
}

void ble_disconnect_and_restart(void)
{
    ESP_LOGI(TAG, "Device reset requested. Notifying mobile app before restart...");

    if (s_is_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    display_set_brightness(0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Rebooting ESP32...");
    esp_restart();
}

// Helper: Notify mobile app of OTA status/progress
static void ble_ota_notify_status(uint8_t status_code, uint8_t progress_pct)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ota_status_val_handle == 0) return;

    uint8_t notify_buf[4] = { status_code, progress_pct, 0, 0 };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(notify_buf, sizeof(notify_buf));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_ota_status_val_handle, om);
    }
}

// =============================================================================
// GATT CURRENT TIME SERVICE CALLBACK
// =============================================================================

static int ble_time_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[32] = {0};
        uint16_t copied_len = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &copied_len);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        buf[copied_len] = '\0';
        ESP_LOGI(TAG, "BLE time sync packet received (%d bytes)", copied_len);

        int h = 0, m = 0, s = 0;
        // 1. ASCII format: "HH:MM:SS" or "HH:MM"
        if (sscanf((char *)buf, "%d:%d:%d", &h, &m, &s) >= 2 ||
            sscanf((char *)buf, "%d:%d", &h, &m) >= 2) {
            ESP_LOGI(TAG, "RTC synchronized from ASCII: %02d:%02d:%02d", h, m, s);
            time_set_current_time((uint8_t)h, (uint8_t)m, (uint8_t)s);
            return 0;
        }

        // 2. Standard Bluetooth SIG Current Time Service (CTS) binary format (7-10 bytes)
        // Byte 0-1: Year (uint16_t LE), Byte 2: Month, Byte 3: Day, Byte 4: Hour, Byte 5: Min, Byte 6: Sec
        if (copied_len >= 7 && copied_len <= 10) {
            uint8_t h_bin = buf[4];
            uint8_t m_bin = buf[5];
            uint8_t s_bin = buf[6];
            uint16_t year = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
            if (time_set_datetime(year, buf[2], buf[3], h_bin, m_bin, s_bin)) {
                ESP_LOGI(TAG, "RTC synchronized from CTS binary: %02d:%02d:%02d", h_bin, m_bin, s_bin);
                return 0;
            }
        }

        // 3. Unix epoch timestamp (4 bytes uint32 LE)
        if (copied_len == 4) {
            uint32_t epoch = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                             ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
            if (epoch > 1700000000) {
                time_set_epoch(epoch);
                return 0;
            }
        }

        // 4. Raw 2 or 3 bytes binary: [HH, MM] or [HH, MM, SS]
        if (copied_len >= 2 && copied_len <= 3) {
            uint8_t raw_h = buf[0];
            uint8_t raw_m = buf[1];
            uint8_t raw_s = (copied_len == 3) ? buf[2] : 0;
            if (raw_h < 24 && raw_m < 60 && raw_s < 60) {
                ESP_LOGI(TAG, "RTC synchronized from raw bytes: %02d:%02d:%02d", raw_h, raw_m, raw_s);
                time_set_current_time(raw_h, raw_m, raw_s);
                return 0;
            }
        }

        ESP_LOGW(TAG, "Unrecognized time sync format (%d bytes)", copied_len);
        return 0;
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        time_t now;
        time(&now);
        struct tm tm_info;
        localtime_r(&now, &tm_info);

        char time_str[16];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
        os_mbuf_append(ctxt->om, time_str, strlen(time_str));
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

// =============================================================================
// GATT OTA SERVICE CALLBACKS
// =============================================================================

static int ble_ota_control_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t cmd_buf[16] = {0};
        uint16_t len = 0;
        ble_hs_mbuf_to_flat(ctxt->om, cmd_buf, sizeof(cmd_buf), &len);

        if (len == 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        uint8_t opcode = cmd_buf[0];

        switch (opcode) {
        case OTA_CMD_START: {
            if (s_img_transfer_active) return BLE_ATT_ERR_UNLIKELY;
            if (len < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            uint32_t image_size = (uint32_t)cmd_buf[1] |
                                  ((uint32_t)cmd_buf[2] << 8) |
                                  ((uint32_t)cmd_buf[3] << 16) |
                                  ((uint32_t)cmd_buf[4] << 24);

            ESP_LOGI(TAG, "OTA CMD: START (Size: %" PRIu32 " bytes)", image_size);
            esp_err_t err = ota_start(image_size);
            if (err == ESP_OK) {
                s_ota_last_reported_pct = 0;
                s_ota_expected_seq = 0;
                ble_ota_notify_status(OTA_STATUS_READY, 0);
            } else {
                display_set_screen(SCREEN_HOME);
                ble_ota_notify_status(OTA_STATUS_ERROR_FLASH_WRITE, 0);
            }
            return 0;
        }

        case OTA_CMD_FINISH: {
            ESP_LOGI(TAG, "OTA CMD: FINISH");
            esp_err_t err = ota_finish();
            if (err == ESP_OK) {
                display_show_ota_screen(100);
                ble_ota_notify_status(OTA_STATUS_SUCCESS, 100);
            } else {
                display_set_screen(SCREEN_HOME);
                ble_ota_notify_status(OTA_STATUS_ERROR_VALIDATION_FAIL, ota_get_progress_percent());
            }
            return 0;
        }

        case OTA_CMD_ABORT: {
            ESP_LOGW(TAG, "OTA CMD: ABORT");
            ota_abort();
            display_set_screen(SCREEN_HOME); // Return back to Home screen safely
            ble_ota_notify_status(OTA_STATUS_ERROR_ABORTED, 0);
            return 0;
        }

        case OTA_CMD_REBOOT: {
            if (ota_get_status() != OTA_STATUS_SUCCESS) {
                ESP_LOGW(TAG, "Ignoring OTA reboot: update has not been verified");
                return BLE_ATT_ERR_UNLIKELY;
            }
            ESP_LOGI(TAG, "OTA CMD: REBOOT. Restarting into new firmware...");
            display_show_ota_screen(100);
            vTaskDelay(pdMS_TO_TICKS(500));
            ble_disconnect_and_restart();
            return 0;
        }

        default:
            ESP_LOGW(TAG, "Unknown OTA Control opcode: 0x%02X", opcode);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int ble_ota_data_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t chunk_buf[512];
        uint16_t len = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, chunk_buf, sizeof(chunk_buf), &len);
        if (rc != 0 || len < 3) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        // Sequence number in first 2 bytes (Little-Endian)
        uint16_t seq = (uint16_t)chunk_buf[0] | ((uint16_t)chunk_buf[1] << 8);
        const uint8_t *payload = &chunk_buf[2];
        uint16_t payload_len = len - 2;

        // A retry after a lost ATT response must not write the same bytes twice.
        if (s_ota_expected_seq > 0 && seq == (uint16_t)(s_ota_expected_seq - 1)) {
            return 0;
        }
        if (seq != s_ota_expected_seq) {
            ESP_LOGW(TAG, "Unexpected OTA chunk sequence: got %u, expected %u",
                     seq, s_ota_expected_seq);
            return BLE_ATT_ERR_UNLIKELY;
        }

        esp_err_t err = ota_write_chunk((uint32_t)seq, payload, payload_len);
        if (err != ESP_OK) {
            display_set_screen(SCREEN_HOME);
            ble_ota_notify_status(OTA_STATUS_ERROR_FLASH_WRITE, ota_get_progress_percent());
            return BLE_ATT_ERR_UNLIKELY;
        }
        s_ota_expected_seq++;

        // Send periodic progress notifications and refresh screen progress (every ~2%)
        uint8_t current_pct = ota_get_progress_percent();
        if (current_pct >= s_ota_last_reported_pct + 2 || current_pct == 100) {
            s_ota_last_reported_pct = current_pct;
            display_show_ota_screen(current_pct);
            ble_ota_notify_status(OTA_STATUS_RECEIVING, current_pct);
        }

        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int ble_ota_status_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t status_buf[4] = {
            (uint8_t)ota_get_status(),
            ota_get_progress_percent(),
            0,
            0
        };
        os_mbuf_append(ctxt->om, status_buf, sizeof(status_buf));
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// =============================================================================
// IMAGE TRANSFER SERVICE CALLBACKS (0xFF20)
// =============================================================================

static void ble_img_notify_status(uint8_t status_code, uint8_t progress_pct)
{
    s_img_status = status_code;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_img_status_val_handle == 0) return;
    uint8_t buf[2] = { status_code, progress_pct };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (om) ble_gatts_notify_custom(s_conn_handle, s_img_status_val_handle, om);
}

// 0xFF21 — Image Control
static int ble_img_control_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    uint8_t cmd_buf[8] = {0};
    uint16_t len = 0;
    ble_hs_mbuf_to_flat(ctxt->om, cmd_buf, sizeof(cmd_buf), &len);
    if (len == 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    switch (cmd_buf[0]) {
    case IMG_CMD_START: {
        uint32_t image_size = (len >= 5)
            ? ((uint32_t)cmd_buf[1] | ((uint32_t)cmd_buf[2] << 8)
              | ((uint32_t)cmd_buf[3] << 16) | ((uint32_t)cmd_buf[4] << 24))
            : IMG_TOTAL_BYTES;

        if (image_size != IMG_TOTAL_BYTES) {
            ESP_LOGW(TAG, "IMG: unexpected size %" PRIu32 " (expected %u)", image_size, (unsigned)IMG_TOTAL_BYTES);
            ble_img_notify_status(IMG_STATUS_ERROR_SIZE, 0);
            return 0;
        }

        if (ota_get_status() == OTA_STATUS_READY || ota_get_status() == OTA_STATUS_RECEIVING)
            return BLE_ATT_ERR_UNLIKELY;
        s_img_expected_seq = 0;
        s_img_bytes_received  = 0;
        s_img_transfer_active = true;
        if (!display_start_image_stream()) {
            s_img_transfer_active = false;
            ble_img_notify_status(IMG_STATUS_ERROR_ABORTED, 0);
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "IMG: START — storing image, expecting %" PRIu32 " bytes", image_size);
        ble_img_notify_status(IMG_STATUS_READY, 0);
        return 0;
    }
    case IMG_CMD_FINISH: {
        if (!s_img_transfer_active || s_img_bytes_received != IMG_TOTAL_BYTES) {
            s_img_transfer_active = false;
            ble_img_notify_status(IMG_STATUS_ERROR_SIZE, 0);
            return BLE_ATT_ERR_UNLIKELY;
        }
        s_img_transfer_active = false;
        uint8_t pct = (uint8_t)((s_img_bytes_received * 100u) / IMG_TOTAL_BYTES);
        ESP_LOGI(TAG, "IMG: FINISH — %" PRIu32 "/%u bytes received (%d%%)",
                 s_img_bytes_received, (unsigned)IMG_TOTAL_BYTES, pct);
        if (!display_finish_image_stream()) {
            ble_img_notify_status(IMG_STATUS_ERROR_SIZE, 0);
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_img_notify_status(IMG_STATUS_SUCCESS, 100);
        return 0;
    }
    case IMG_CMD_ABORT:
        s_img_transfer_active = false;
        s_img_bytes_received  = 0;
        ESP_LOGW(TAG, "IMG: ABORT");
        display_set_screen(SCREEN_NAV);
        ble_img_notify_status(IMG_STATUS_ERROR_ABORTED, 0);
        return 0;
    case IMG_CMD_RESET:
        s_img_transfer_active = false;
        s_img_bytes_received  = 0;
        ESP_LOGI(TAG, "IMG: RESET — clearing custom image and resetting to default");
        display_set_screen(SCREEN_NAV);
        ble_img_notify_status(IMG_STATUS_READY, 0);
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// 0xFF22 — Image Data: [seq_lo][seq_hi][...RGB565 payload...]
static int ble_img_data_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    if (!s_img_transfer_active) return BLE_ATT_ERR_UNLIKELY;

    uint8_t chunk_buf[2 + IMG_CHUNK_SIZE];
    uint16_t len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, chunk_buf, sizeof(chunk_buf), &len);
    if (rc != 0 || len < 3) return BLE_ATT_ERR_UNLIKELY;

    const uint8_t *payload = &chunk_buf[2];
    uint16_t       pay_len = len - 2;

    uint16_t seq = (uint16_t)chunk_buf[0] | ((uint16_t)chunk_buf[1] << 8);
    if (s_img_expected_seq > 0 && seq == s_img_expected_seq - 1) return 0;
    if (seq != s_img_expected_seq || (pay_len & 1) ||
        s_img_bytes_received + pay_len > IMG_TOTAL_BYTES) return BLE_ATT_ERR_UNLIKELY;
    s_img_expected_seq++;
    if (!display_write_image_chunk(payload, pay_len)) {
        s_img_transfer_active = false;
        ble_img_notify_status(IMG_STATUS_ERROR_ABORTED, 0);
        return BLE_ATT_ERR_UNLIKELY;
    }
    s_img_bytes_received += pay_len;

    // Notify progress every ~5%
    static uint8_t last_img_pct = 0;
    uint8_t pct = (uint8_t)((s_img_bytes_received * 100u) / IMG_TOTAL_BYTES);
    if (pct >= last_img_pct + 5 || s_img_bytes_received >= IMG_TOTAL_BYTES) {
        last_img_pct = pct;
        ble_img_notify_status(IMG_STATUS_RECEIVING, pct);
    }
    return 0;
}

// 0xFF23 — Image Status (readable)
static int ble_img_status_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t buf[2] = {
            s_img_status,
            (uint8_t)((s_img_bytes_received * 100u) / IMG_TOTAL_BYTES)
        };
        os_mbuf_append(ctxt->om, buf, sizeof(buf));
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// =============================================================================
// NAVIGATION SERVICE CALLBACKS (0xFF30)
// =============================================================================

static nav_direction_t ble_code_to_nav_direction(uint8_t code) {
    switch (code) {
        case 0: return NAV_DIR_NONE;
        case 1: return NAV_DIR_STRAIGHT;
        case 2: return NAV_DIR_SLIGHT_LEFT;
        case 3: return NAV_DIR_SLIGHT_RIGHT;
        case 4: return NAV_DIR_LEFT;
        case 5: return NAV_DIR_RIGHT;
        case 6: return NAV_DIR_SHARP_LEFT;
        case 7: return NAV_DIR_SHARP_RIGHT;
        case 8: return NAV_DIR_UTURN;
        case 9: return NAV_DIR_REACHED;
        case 10: return NAV_DIR_ROUNDABOUT;
        case 11: return NAV_DIR_EXIT;
        case 12: return NAV_DIR_MERGE;
        case 13: return NAV_DIR_FORK;
        default:
            if (code >= NAV_DIR_STRAIGHT && code <= NAV_DIR_FORK) {
                return (nav_direction_t)code;
            }
            return NAV_DIR_STRAIGHT;
    }
}

// 0xFF31 — Navigation Data (Binary or JSON packet)
static int ble_nav_data_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    uint8_t rx_buf[256];
    uint16_t len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, rx_buf, sizeof(rx_buf) - 1, &len);
    if (rc != 0 || len == 0) return BLE_ATT_ERR_UNLIKELY;

    if (rx_buf[0] == 0x13) {
        if (len < 6) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint32_t revision = (uint32_t)rx_buf[1] | ((uint32_t)rx_buf[2] << 8) |
            ((uint32_t)rx_buf[3] << 16) | ((uint32_t)rx_buf[4] << 24);
        uint8_t op = rx_buf[5];
        if (!((op >= NAV_OP_START && op <= NAV_OP_STOP) || op == NAV_OP_SYNC ||
              op == NAV_OP_ROUTE_SEGMENT || op == NAV_OP_CONTEXT || (op >= 0x14 && op <= 0x19)))
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        if (s_revision_received && (int32_t)(revision - s_route_revision) < 0) return BLE_ATT_ERR_UNLIKELY;
        if (!s_revision_received || revision != s_route_revision) {
            // Route changes invalidate geometry, but keep the visible old route
            // until its replacement arrives; the next segment replaces it.
            display_begin_route_revision();
            s_route_revision = revision;
            s_revision_received = true;
        }
        memmove(rx_buf, rx_buf + 5, len - 5);
        len -= 5;
    }
    if(rx_buf[0]==0x1C) {
        if(len<4 || rx_buf[1]>2 || rx_buf[2]>3 || rx_buf[3]>47 || len!=(size_t)(4+rx_buf[3]))
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        char caller[48]={0};memcpy(caller,rx_buf+4,rx_buf[3]);
        display_update_call(rx_buf[1],rx_buf[2],caller);return 0;
    }
    if (rx_buf[0] == 0x1A) {
        if (len != 2 || rx_buf[1] > 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        if (rx_buf[1] == 1 && !mpu6500_request_calibration()) {
            mpu6500_publish_status();
            return BLE_ATT_ERR_UNLIKELY;
        }
        mpu6500_publish_status();
        return 0;
    }
    if ((rx_buf[0] >= 0x14 && rx_buf[0] <= 0x19)) {
        if (s_img_transfer_active || (ota_get_status() == OTA_STATUS_READY || ota_get_status() == OTA_STATUS_RECEIVING)) return BLE_ATT_ERR_UNLIKELY;
        return display_map_packet(rx_buf, len) ? 0 : BLE_ATT_ERR_UNLIKELY;
    }
    if (rx_buf[0] == 0x12) {
        if (len < 6 || rx_buf[1] > 7 || rx_buf[2] > 31 || rx_buf[3] > 100)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        size_t title_len = rx_buf[4];
        if (title_len > 40 || len < 6 + title_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        size_t artist_len = rx_buf[5 + title_len];
        if (artist_len > 32 || len != 6 + title_len + artist_len)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        media_state_t media = {0};
        media.state = rx_buf[1]; media.actions = rx_buf[2]; media.volume = rx_buf[3];
        memcpy(media.title, rx_buf + 5, title_len);
        memcpy(media.artist, rx_buf + 6 + title_len, artist_len);
        for (size_t i = 0; i < title_len; i++)
            if (media.title[i] < 32 || media.title[i] > 126) media.title[i] = '?';
        for (size_t i = 0; i < artist_len; i++)
            if (media.artist[i] < 32 || media.artist[i] > 126) media.artist[i] = '?';
        display_update_media(&media);
        return 0;
    }

    rx_buf[len] = '\0';

    navigation_data_t nav_data = {0};

    // 1. JSON-formatted packet (Simulator or raw debug mode)
    if (rx_buf[0] == '{') {
        char maneuver_str[32] = {0};
        char street_str[32] = {0};
        uint32_t maneuver_dist = 0;
        uint32_t speed = 0;
        uint32_t rem_dist = 0;
        uint32_t eta_secs = 0;
        uint32_t maneuver_id = 0;
        uint32_t maneuver_state = 0;

        char *p_man = strstr((char *)rx_buf, "\"maneuver\":");
        if (p_man) sscanf(p_man, "\"maneuver\":\"%31[^\"]\"", maneuver_str);

        char *p_mid = strstr((char *)rx_buf, "\"maneuverId\":");
        if (p_mid) sscanf(p_mid, "\"maneuverId\":%" PRIu32, &maneuver_id);

        char *p_mst = strstr((char *)rx_buf, "\"maneuverState\":");
        if (p_mst) sscanf(p_mst, "\"maneuverState\":%" PRIu32, &maneuver_state);

        char *p_mdist = strstr((char *)rx_buf, "\"maneuverDistance\":");
        if (p_mdist) sscanf(p_mdist, "\"maneuverDistance\":%" PRIu32, &maneuver_dist);

        char *p_spd = strstr((char *)rx_buf, "\"speed\":");
        if (p_spd) sscanf(p_spd, "\"speed\":%" PRIu32, &speed);

        char *p_rem = strstr((char *)rx_buf, "\"remainingDistance\":");
        if (p_rem) sscanf(p_rem, "\"remainingDistance\":%" PRIu32, &rem_dist);

        char *p_eta = strstr((char *)rx_buf, "\"eta\":");
        if (p_eta) sscanf(p_eta, "\"eta\":%" PRIu32, &eta_secs);

        char *p_st = strstr((char *)rx_buf, "\"street\":");
        if (p_st) sscanf(p_st, "\"street\":\"%31[^\"]\"", street_str);

        // Direction mapping
        nav_data.direction = NAV_DIR_STRAIGHT;
        if (strstr(maneuver_str, "LEFT")) {
            nav_data.direction = strstr(maneuver_str, "SHARP") ? NAV_DIR_SHARP_LEFT :
                                 strstr(maneuver_str, "SLIGHT") ? NAV_DIR_SLIGHT_LEFT : NAV_DIR_LEFT;
        } else if (strstr(maneuver_str, "RIGHT")) {
            nav_data.direction = strstr(maneuver_str, "SHARP") ? NAV_DIR_SHARP_RIGHT :
                                 strstr(maneuver_str, "SLIGHT") ? NAV_DIR_SLIGHT_RIGHT : NAV_DIR_RIGHT;
        } else if (strstr(maneuver_str, "U_TURN") || strstr(maneuver_str, "UTURN")) {
            nav_data.direction = NAV_DIR_UTURN;
        } else if (strstr(maneuver_str, "ROUNDABOUT")) {
            nav_data.direction = NAV_DIR_ROUNDABOUT;
        } else if (strstr(maneuver_str, "EXIT")) {
            nav_data.direction = NAV_DIR_EXIT;
        } else if (strstr(maneuver_str, "MERGE")) {
            nav_data.direction = NAV_DIR_MERGE;
        } else if (strstr(maneuver_str, "FORK")) {
            nav_data.direction = NAV_DIR_FORK;
        } else if (strstr(maneuver_str, "ARRIVE") || strstr(maneuver_str, "REACHED")) {
            nav_data.direction = NAV_DIR_REACHED;
        }

        nav_data.maneuver_id = (uint16_t)maneuver_id;
        nav_data.maneuver_state = (uint8_t)maneuver_state;
        nav_data.distance_meters = maneuver_dist;
        nav_data.speed_kmh = speed;
        nav_data.remaining_dist_km = (float)rem_dist / 1000.0f;
        nav_data.is_navigating = nav_data.direction != NAV_DIR_REACHED;
        nav_data.gps_available = true;
        snprintf(nav_data.eta_time_str, sizeof(nav_data.eta_time_str), "%lu min", (unsigned long)(eta_secs / 60));
        strncpy(nav_data.current_street, street_str[0] ? street_str : "Motorcycle Route", sizeof(nav_data.current_street) - 1);

        char next_maneuver_str[32] = {0};
        char next_street_str[32] = {0};
        uint32_t next_maneuver_dist = 0;

        char *p_nman = strstr((char *)rx_buf, "\"nextManeuver\":");
        if (p_nman) sscanf(p_nman, "\"nextManeuver\":\"%31[^\"]\"", next_maneuver_str);

        char *p_ndist = strstr((char *)rx_buf, "\"nextManeuverDistance\":");
        if (p_ndist) sscanf(p_ndist, "\"nextManeuverDistance\":%" PRIu32, &next_maneuver_dist);

        char *p_nst = strstr((char *)rx_buf, "\"nextStreet\":");
        if (p_nst) sscanf(p_nst, "\"nextStreet\":\"%31[^\"]\"", next_street_str);

        nav_data.next_direction = NAV_DIR_NONE;
        if (next_maneuver_str[0]) {
            if (strstr(next_maneuver_str, "LEFT")) {
                nav_data.next_direction = strstr(next_maneuver_str, "SHARP") ? NAV_DIR_SHARP_LEFT :
                                          strstr(next_maneuver_str, "SLIGHT") ? NAV_DIR_SLIGHT_LEFT : NAV_DIR_LEFT;
            } else if (strstr(next_maneuver_str, "RIGHT")) {
                nav_data.next_direction = strstr(next_maneuver_str, "SHARP") ? NAV_DIR_SHARP_RIGHT :
                                          strstr(next_maneuver_str, "SLIGHT") ? NAV_DIR_SLIGHT_RIGHT : NAV_DIR_RIGHT;
            } else if (strstr(next_maneuver_str, "U_TURN") || strstr(next_maneuver_str, "UTURN")) {
                nav_data.next_direction = NAV_DIR_UTURN;
            } else if (strstr(next_maneuver_str, "ROUNDABOUT")) {
                nav_data.next_direction = NAV_DIR_ROUNDABOUT;
            } else if (strstr(next_maneuver_str, "EXIT")) {
                nav_data.next_direction = NAV_DIR_EXIT;
            } else if (strstr(next_maneuver_str, "MERGE")) {
                nav_data.next_direction = NAV_DIR_MERGE;
            } else if (strstr(next_maneuver_str, "FORK")) {
                nav_data.next_direction = NAV_DIR_FORK;
            } else if (strstr(next_maneuver_str, "ARRIVE") || strstr(next_maneuver_str, "REACHED")) {
                nav_data.next_direction = NAV_DIR_REACHED;
            } else {
                nav_data.next_direction = NAV_DIR_STRAIGHT;
            }
        }
        nav_data.next_distance_meters = next_maneuver_dist;
        strncpy(nav_data.next_street, next_street_str, sizeof(nav_data.next_street) - 1);

        display_update_navigation(&nav_data);
        return 0;
    }

    // 2. Direct Device Control Commands from Mobile App (Brightness, Restart, Reset, Cancel/Stop)
    if (len >= 1) {
        uint8_t op = rx_buf[0];
        if (op == NAV_OP_SET_BRIGHTNESS && len >= 2) {
            uint8_t b_pct = rx_buf[1];
            if (b_pct > 100) b_pct = 100;
            ESP_LOGI(TAG, "BLE Remote: Set Brightness -> %d%%", b_pct);
            display_set_brightness(b_pct);
            return 0;
        } else if (op == NAV_OP_RESTART && len == 1) {
            ESP_LOGW(TAG, "BLE Remote: Restart requested from App");
            ble_disconnect_and_restart();
            return 0;
        } else if (op == NAV_OP_RESET) {
            ESP_LOGW(TAG, "BLE Remote: Factory Reset requested from App");
            display_set_brightness(70);
            display_nav_recenter();
            display_set_screen(SCREEN_HOME);
            return 0;
        } else if ((op == NAV_OP_STOP || op == NAV_OP_CANCEL) && len < 18) {
            ESP_LOGI(TAG, "BLE Remote: Navigation Cancel/Stop requested from App");
            navigation_data_t stop_data = {0};
            stop_data.is_navigating = false;
            stop_data.speed_kmh = 0;
            strncpy(stop_data.current_street, "Navigation Ended", sizeof(stop_data.current_street) - 1);
            display_update_navigation(&stop_data);
            return 0;
        } else if (op == NAV_OP_NOTIFICATION && len >= 3) {
            // Legacy title/message followed by optional v3 app-label metadata.
            size_t at=2;uint8_t tl=rx_buf[1];
            if(tl>31 || at+tl>=len)return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            char title[32]={0},message[64]={0},app[24]="Alert";
            memcpy(title,rx_buf+at,tl);at+=tl;
            uint8_t ml=rx_buf[at++];
            if(ml>63 || at+ml>len)return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            memcpy(message,rx_buf+at,ml);at+=ml;
            if(at<len){uint8_t al=rx_buf[at++];if(al>23 || at+al!=len)return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
              memset(app,0,sizeof(app));memcpy(app,rx_buf+at,al);}
            display_show_app_notification(app,title,message);
            return 0;
        } else if (op == NAV_OP_DIAGNOSTICS) {
            ESP_LOGI(TAG, "BLE Remote: Diagnostics self-test requested from App");
            uint8_t diag_buf[7] = {
                NAV_STATUS_DIAGNOSTICS, // 0x10
                1,                      // Bluetooth link OK
                0xFF,                   // GPS health not measured by this device
                0xFF,                   // Display health not measured
                0xFF,                   // Battery measurement unavailable
                0xFF,                   // Power source unknown
                0xFF                    // Voltage unknown
            };
            ble_nav_notify_diagnostic_packet(diag_buf, sizeof(diag_buf));
            return 0;
        }
    }

    // Optional low-rate city and nearby place context (OpCode 0x11).
    if (len >= 10 && rx_buf[0] == NAV_OP_CONTEXT) {
        size_t city_len = rx_buf[1];
        if (city_len > 18 || len < 10 + city_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        size_t offset = 2 + city_len;
        size_t name_len = rx_buf[offset + 7];
        if (name_len > 18 || len < offset + 8 + name_len)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        nav_context_t context;
        memset(&context, 0, sizeof(context));
        memcpy(context.city, &rx_buf[2], city_len);
        context.poi_type = rx_buf[offset];
        context.poi_distance_m = (uint16_t)rx_buf[offset + 1] |
                                 ((uint16_t)rx_buf[offset + 2] << 8);
        context.poi_x = (uint16_t)rx_buf[offset + 3] |
                        ((uint16_t)rx_buf[offset + 4] << 8);
        context.poi_y = (uint16_t)rx_buf[offset + 5] |
                        ((uint16_t)rx_buf[offset + 6] << 8);
        memcpy(context.poi_name, &rx_buf[offset + 8], name_len);
        display_update_nav_context(&context);
        return 0;
    }

    // 3. Route Segment packet format (OpCode 0x0B)
    if (len >= 7 && rx_buf[0] == NAV_OP_ROUTE_SEGMENT) {
        uint8_t count = rx_buf[1];
        if (count > ROUTE_MAX_POINTS) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        if (len >= (size_t)(7 + count * 4)) {
            route_segment_t seg;
            memset(&seg, 0, sizeof(seg));
            seg.count = count;
            seg.is_destination_visible = (rx_buf[2] & 0x01) != 0;
            seg.completed_segment_count = (rx_buf[2] >> 1) & 0x0f;
            if (seg.completed_segment_count >= count)
                seg.completed_segment_count = count ? count - 1 : 0;
            seg.progress_percent = len > (size_t)(7 + count * 4)
                ? rx_buf[7 + count * 4] : 0;
            if (seg.progress_percent > 100) seg.progress_percent = 100;
            seg.dest_x = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            seg.dest_y = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
            size_t offset = 7;
            for (uint8_t i = 0; i < count; i++) {
                seg.points[i].x = (uint16_t)rx_buf[offset] | ((uint16_t)rx_buf[offset + 1] << 8);
                seg.points[i].y = (uint16_t)rx_buf[offset + 2] | ((uint16_t)rx_buf[offset + 3] << 8);
                if (seg.points[i].x > 320 || seg.points[i].y > 240)
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                offset += 4;
            }
            ESP_LOGI(TAG, "NAV Route Segment: %d pts, dest_vis=%d (%d,%d)",
                     count, seg.is_destination_visible, seg.dest_x, seg.dest_y);
            display_update_route_segment(&seg);
            return 0;
        }
    }

    // 4. Compact Binary packet format (Production mode)
    if (len >= 18 && ((rx_buf[0] >= NAV_OP_START && rx_buf[0] <= NAV_OP_STOP) ||
                      rx_buf[0] == NAV_OP_SYNC)) {
        uint8_t opcode = rx_buf[0];
        uint8_t maneuver = rx_buf[1];
        uint32_t maneuver_dist = (uint32_t)rx_buf[2] | ((uint32_t)rx_buf[3] << 8);
        uint32_t speed = (uint32_t)rx_buf[4] | ((uint32_t)rx_buf[5] << 8);
        uint32_t heading = (uint32_t)rx_buf[6] | ((uint32_t)rx_buf[7] << 8);
        uint32_t rem_dist = (uint32_t)rx_buf[8] | ((uint32_t)rx_buf[9] << 8) |
                            ((uint32_t)rx_buf[10] << 16) | ((uint32_t)rx_buf[11] << 24);
        uint32_t rem_dur = (uint32_t)rx_buf[12] | ((uint32_t)rx_buf[13] << 8) |
                           ((uint32_t)rx_buf[14] << 16) | ((uint32_t)rx_buf[15] << 24);
        uint8_t flags = rx_buf[16];

        uint8_t maneuver_state = 0;
        uint16_t maneuver_id = 0;
        uint8_t text_len = 0;
        char street_buf[32] = {0};

        if (len >= 21) {
            maneuver_state = rx_buf[17];
            maneuver_id = (uint16_t)rx_buf[18] | ((uint16_t)rx_buf[19] << 8);
            text_len = rx_buf[20];
            if (text_len > 31 || len < 21 + text_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            if (text_len > 0 && len >= (size_t)(21 + text_len)) {
                int copy_bytes = (text_len < 31) ? text_len : 31;
                memcpy(street_buf, &rx_buf[21], copy_bytes);
                street_buf[copy_bytes] = '\0';
            }
        } else {
            // Legacy 18-byte header
            text_len = rx_buf[17];
            if (text_len > 31 || len < 18 + text_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            if (text_len > 0 && len >= (size_t)(18 + text_len)) {
                int copy_bytes = (text_len < 31) ? text_len : 31;
                memcpy(street_buf, &rx_buf[18], copy_bytes);
                street_buf[copy_bytes] = '\0';
            }
        }

        ESP_LOGI(TAG, "NAV Binary: Op=0x%02X Man=%u State=%u Id=%u Dist=%" PRIu32 "m Spd=%" PRIu32 "km/h Hdg=%" PRIu32 " Rem=%" PRIu32 "m Street=%s",
                 opcode, maneuver, maneuver_state, maneuver_id, maneuver_dist, speed, heading, rem_dist, street_buf);

        if (opcode == NAV_OP_STOP || opcode == NAV_OP_CANCEL) {
            nav_data.is_navigating = false;
            nav_data.speed_kmh = 0;
            strncpy(nav_data.current_street, "Navigation Ended", sizeof(nav_data.current_street) - 1);
            display_update_navigation(&nav_data);
            return 0;
        }

        bool is_nav = (flags & (1 << 0)) != 0;
        bool is_reroute = (flags & (1 << 1)) != 0 || (opcode == NAV_OP_REROUTE);
        bool gps_ok = (flags & (1 << 2)) != 0;

        nav_data.direction = ble_code_to_nav_direction(maneuver);
        nav_data.maneuver_state = maneuver_state;
        nav_data.maneuver_id = maneuver_id;
        nav_data.distance_meters = maneuver_dist;
        nav_data.speed_kmh = gps_ok ? (uint16_t)speed : 0;
        nav_data.heading_deg = heading;
        nav_data.remaining_dist_km = (float)rem_dist / 1000.0f;
        nav_data.is_navigating = is_nav && (opcode != NAV_OP_ARRIVED);
        nav_data.gps_available = gps_ok;
        nav_data.rerouting = is_reroute;

        if (opcode == NAV_OP_REROUTE || is_reroute) {
            strncpy(nav_data.current_street, "REROUTING...", sizeof(nav_data.current_street) - 1);
        } else if (opcode == NAV_OP_ARRIVED) {
            strncpy(nav_data.current_street, "DESTINATION REACHED", sizeof(nav_data.current_street) - 1);
        } else {
            strncpy(nav_data.current_street, street_buf[0] ? street_buf : "Motorcycle Route", sizeof(nav_data.current_street) - 1);
        }

        if (rem_dur >= 60) {
            uint16_t rem_mins = (uint16_t)((rem_dur / 60) > 9999u ? 9999u : (rem_dur / 60));
            snprintf(nav_data.eta_time_str, sizeof(nav_data.eta_time_str), "%u min", (unsigned)rem_mins);
        } else {
            snprintf(nav_data.eta_time_str, sizeof(nav_data.eta_time_str), "< 1 min");
        }

        // Lookahead / Secondary maneuver parsing
        size_t next_offset = (len >= 21 ? 21 : 18) + text_len;
        if (len > next_offset && len < next_offset + 4) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        if (len >= next_offset + 4) {
            uint8_t next_man = rx_buf[next_offset];
            uint16_t next_dist = (uint16_t)rx_buf[next_offset + 1] | ((uint16_t)rx_buf[next_offset + 2] << 8);
            char next_street_buf[32] = {0};
            if (len >= next_offset + 4) {
                uint8_t next_txt_len = rx_buf[next_offset + 3];
                if (next_txt_len > 31 || len != next_offset + 4 + next_txt_len)
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                if (next_txt_len > 0 && len >= next_offset + 4 + next_txt_len) {
                    int copy_n = (next_txt_len < 31) ? next_txt_len : 31;
                    memcpy(next_street_buf, &rx_buf[next_offset + 4], copy_n);
                    next_street_buf[copy_n] = '\0';
                }
            }
            nav_data.next_direction = ble_code_to_nav_direction(next_man);
            nav_data.next_distance_meters = next_dist;
            strncpy(nav_data.next_street, next_street_buf, sizeof(nav_data.next_street) - 1);
        }

        // A real route start owns the HUD. Switch to SCREEN_NAV whenever
        // active navigation is running so the rider immediately sees turn directions.
        display_update_navigation(&nav_data);
        return 0;
    }

    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
}

// 0xFF32 — Navigation Status / Button Feedback
static int ble_nav_status_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t status_buf[3] = { 0x01, 0x03, 0x03 }; // Sensor and raster HUD
        os_mbuf_append(ctxt->om, status_buf, sizeof(status_buf));
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

void ble_media_command(uint8_t command)
{
    if (command < 1 || command > 8) return;
    ESP_LOGI(TAG, "MEDIA command -> phone: %u", command);
    uint8_t packet[2] = {0x12, command};
    ble_nav_notify_diagnostic_packet(packet, sizeof(packet));
}

void ble_nav_notify_status(uint8_t status_code)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_nav_status_val_handle == 0) return;
    uint8_t buf[2] = { status_code, 0 };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (om) ble_gatts_notify_custom(s_conn_handle, s_nav_status_val_handle, om);
}

void ble_nav_notify_diagnostic_packet(const uint8_t *data, uint16_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_nav_status_val_handle == 0 || !data || len == 0) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) ble_gatts_notify_custom(s_conn_handle, s_nav_status_val_handle, om);
}


// =============================================================================
// GATT SERVICES DEFINITION TABLE
// =============================================================================

static const struct ble_gatt_svc_def gatt_svcs[] = {
    // 1. Standard Current Time Service (CTS)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_CURRENT_TIME_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_CURRENT_TIME_UUID16),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_time_chr_access,
            },
            { 0 }
        },
    },

    // 2. JustRide OTA Firmware Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // OTA Control
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_OTA_CONTROL_UUID16),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_ota_control_access,
            },
            {
                // OTA Data (High-speed binary chunk sink)
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_OTA_DATA_UUID16),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_ota_data_access,
            },
            {
                // OTA Status / Progress Notifier
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_OTA_STATUS_UUID16),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_ota_status_val_handle,
                .access_cb = ble_ota_status_access,
            },
            { 0 }
        },
    },

    // 3. JustRide Image Transfer Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_IMG_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Image Control (START / FINISH / ABORT)
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_IMG_CONTROL_UUID16),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_img_control_access,
            },
            {
                // Image Data — high-speed RGB565 chunk sink
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_IMG_DATA_UUID16),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_img_data_access,
            },
            {
                // Image Status / Progress Notifier
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_IMG_STATUS_UUID16),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_img_status_val_handle,
                .access_cb = ble_img_status_access,
            },
            { 0 }
        },
    },

    // 4. JustRide Navigation Service (0xFF30)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_NAV_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Navigation Data (Sink for NAV_START, NAV_UPDATE, NAV_REROUTE, etc.)
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_NAV_DATA_UUID16),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = ble_nav_data_access,
            },
            {
                // Navigation Device Status / Button Action Feedback
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_NAV_STATUS_UUID16),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_nav_status_val_handle,
                .access_cb = ble_nav_status_access,
            },
            { 0 }
        },
    },

    { 0 } // End of table
};

// =============================================================================
// BLE ADVERTISING FUNCTION & RECOVERY TIMER
// =============================================================================

static void adv_timer_cb(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Executing scheduled BLE advertising start...");
    ble_start_advertising();
}

void ble_start_advertising(void)
{
    if (s_is_connected) {
        ESP_LOGD(TAG, "Device currently connected; skipping advertising start.");
        return;
    }

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Advertise CTS, OTA, Image Transfer & Navigation Services
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(BLE_SVC_CURRENT_TIME_UUID16),
        BLE_UUID16_INIT(BLE_SVC_OTA_UUID16),
        BLE_UUID16_INIT(BLE_SVC_IMG_UUID16),
        BLE_UUID16_INIT(BLE_SVC_NAV_UUID16)
    };
    fields.num_uuids16 = 4;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement fields; rc=%d", rc);
        return;
    }

    // Also set Scan Response fields with full device name to ensure Android active scans always get "JUSTRIDE"
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (uint8_t *)BLE_DEVICE_NAME;
    rsp_fields.name_len = strlen(BLE_DEVICE_NAME);
    rsp_fields.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ESP_LOGI(TAG, "JUSTRIDE starting advertisement (CTS + OTA + Nav Service)...");

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == BLE_HS_EALREADY) {
        ESP_LOGD(TAG, "Advertising already active.");
    } else if (rc == BLE_HS_EBUSY) {
        ESP_LOGW(TAG, "NimBLE busy during adv start (rc=EBUSY); scheduling retry in 120ms...");
        if (s_adv_timer) {
            xTimerStart(s_adv_timer, pdMS_TO_TICKS(120));
        }
    } else if (rc != 0) {
        ESP_LOGW(TAG, "Error starting advertising; rc=%d. Will retry in 250ms...", rc);
        if (s_adv_timer) {
            xTimerStart(s_adv_timer, pdMS_TO_TICKS(250));
        }
    } else {
        ESP_LOGI(TAG, "JUSTRIDE advertising started successfully!");
    }
}

// =============================================================================
// BLE GAP EVENT CALLBACK
// =============================================================================

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_is_connected = true;
            s_revision_received = false;
            if (s_adv_timer) xTimerStop(s_adv_timer, 0);
            ESP_LOGI(TAG, "Phone connected (conn_handle=%d)! Services online.", s_conn_handle);
            display_set_ble_status(true);

            // Update BLE connection parameters for resilient background link stability:
            // A longer supervision timeout tolerates brief Android pauses while
            // opening a shared destination or switching between apps.
            struct ble_gap_upd_params conn_params;
            memset(&conn_params, 0, sizeof(conn_params));
            conn_params.itvl_min = 24;            // 30 ms (24 * 1.25ms)
            conn_params.itvl_max = 40;            // 50 ms (40 * 1.25ms)
            conn_params.latency = 0;              // Low latency for snappy turn instructions
            conn_params.supervision_timeout = 2000; // 20000 ms (2000 * 10ms)
            conn_params.min_ce_len = 0;
            conn_params.max_ce_len = 0;
            ble_gap_update_params(s_conn_handle, &conn_params);
        } else {
            ESP_LOGW(TAG, "Connection failed: status=%d", event->connect.status);
            s_is_connected = false;
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            display_set_ble_status(false);
            if (s_adv_timer) {
                xTimerStart(s_adv_timer, pdMS_TO_TICKS(120));
            } else {
                ble_start_advertising();
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        time_save_checkpoint();
        if (s_img_transfer_active) s_img_status = IMG_STATUS_ERROR_ABORTED;
        s_img_transfer_active = false;
        s_revision_received = false;
        s_is_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        if (ota_get_status() == OTA_STATUS_READY ||
            ota_get_status() == OTA_STATUS_RECEIVING) {
            ota_abort();
            display_set_screen(SCREEN_HOME);
        }
        ESP_LOGW(TAG, "Phone disconnected! Reason=0x%04X (%d). Resuming advertising in 120ms...",
                 event->disconnect.reason, event->disconnect.reason);
        display_set_ble_status(false);

        // Resuming advertising after 120ms ensures NimBLE host finishes HCI disconnect cleanup
        if (s_adv_timer != NULL) {
            xTimerStart(s_adv_timer, pdMS_TO_TICKS(120));
        } else {
            ble_start_advertising();
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "BLE MTU updated to: %d bytes", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        s_is_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        if (s_adv_timer) {
            xTimerStart(s_adv_timer, pdMS_TO_TICKS(100));
        } else {
            ble_start_advertising();
        }
        break;

    default:
        break;
    }

    return 0;
}

// =============================================================================
// BLE HOST SYNCHRONIZATION CALLBACK
// =============================================================================

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining BLE address type; rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(s_own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    }

    ESP_LOGI(TAG, "BLE initialized");
    ble_start_advertising();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_init(void)
{
    ESP_LOGI(TAG, "JustRide BLE starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE port: %d", ret);
        return;
    }

    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_on_sync;

    if (!s_adv_timer) {
        s_adv_timer = xTimerCreate("adv_timer", pdMS_TO_TICKS(120), pdFALSE, NULL, adv_timer_cb);
    }

    nimble_port_freertos_init(ble_host_task);
}
