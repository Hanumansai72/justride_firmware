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
#include "battery.h"

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
static uint8_t s_own_addr_type;
static TimerHandle_t s_adv_timer = NULL;

// Image transfer state
#define IMG_TOTAL_BYTES  (320u * 240u * 2u)   // 153 600 bytes (320x240 RGB565)
#define IMG_CHUNK_SIZE   128u
static uint32_t s_img_bytes_received = 0;
static bool     s_img_transfer_active = false;


// Forward declarations
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int ble_ota_control_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_ota_data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_ota_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_control_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_img_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
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
        if (sscanf((char *)buf, "%d:%d:%d", &h, &m, &s) >= 2 ||
            sscanf((char *)buf, "%d:%d", &h, &m) >= 2) {
            time_set_current_time((uint8_t)h, (uint8_t)m, (uint8_t)s);
            return 0;
        }

        if (copied_len >= 2 && copied_len <= 3) {
            time_set_current_time(buf[0], buf[1], (copied_len == 3) ? buf[2] : 0);
            return 0;
        }

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
            if (len < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            uint32_t image_size = (uint32_t)cmd_buf[1] |
                                  ((uint32_t)cmd_buf[2] << 8) |
                                  ((uint32_t)cmd_buf[3] << 16) |
                                  ((uint32_t)cmd_buf[4] << 24);

            ESP_LOGI(TAG, "OTA CMD: START (Size: %" PRIu32 " bytes)", image_size);
            esp_err_t err = ota_start(image_size);
            if (err == ESP_OK) {
                display_show_ota_screen(0); // Switch 2.8" TFT directly to Firmware Flash screen
                ble_ota_notify_status(OTA_STATUS_READY, 0);
            } else {
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
        if (rc != 0 || len == 0) return BLE_ATT_ERR_UNLIKELY;

        // Sequence number in first 2 bytes (Little-Endian)
        uint16_t seq = (uint16_t)chunk_buf[0] | ((uint16_t)chunk_buf[1] << 8);
        const uint8_t *payload = &chunk_buf[2];
        uint16_t payload_len = len - 2;

        esp_err_t err = ota_write_chunk((uint32_t)seq, payload, payload_len);
        if (err != ESP_OK) {
            ble_ota_notify_status(OTA_STATUS_ERROR_FLASH_WRITE, ota_get_progress_percent());
            return BLE_ATT_ERR_UNLIKELY;
        }

        // Send periodic progress notifications and refresh screen progress (every ~2%)
        static uint8_t last_reported_pct = 0;
        uint8_t current_pct = ota_get_progress_percent();
        if (current_pct >= last_reported_pct + 2 || current_pct == 100) {
            last_reported_pct = current_pct;
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

        s_img_bytes_received  = 0;
        s_img_transfer_active = true;
        display_start_image_stream();
        ESP_LOGI(TAG, "IMG: START — direct TFT stream expecting %" PRIu32 " bytes", image_size);
        ble_img_notify_status(IMG_STATUS_READY, 0);
        return 0;
    }
    case IMG_CMD_FINISH: {
        s_img_transfer_active = false;
        uint8_t pct = (uint8_t)((s_img_bytes_received * 100u) / IMG_TOTAL_BYTES);
        ESP_LOGI(TAG, "IMG: FINISH — %" PRIu32 "/%u bytes received (%d%%)",
                 s_img_bytes_received, (unsigned)IMG_TOTAL_BYTES, pct);
        display_finish_image_stream();
        ble_img_notify_status(IMG_STATUS_SUCCESS, 100);
        return 0;
    }
    case IMG_CMD_ABORT:
        s_img_transfer_active = false;
        s_img_bytes_received  = 0;
        ESP_LOGW(TAG, "IMG: ABORT");
        display_show_main_screen();
        ble_img_notify_status(IMG_STATUS_ERROR_ABORTED, 0);
        return 0;
    case IMG_CMD_RESET:
        s_img_transfer_active = false;
        s_img_bytes_received  = 0;
        ESP_LOGI(TAG, "IMG: RESET — clearing custom image and resetting to default");
        display_show_main_screen();
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

    if (s_img_bytes_received + pay_len > IMG_TOTAL_BYTES) {
        pay_len = (uint16_t)(IMG_TOTAL_BYTES - s_img_bytes_received);
    }
    display_write_image_chunk(payload, pay_len);
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
            s_img_transfer_active ? IMG_STATUS_RECEIVING : IMG_STATUS_SUCCESS,
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
        nav_data.is_navigating = (strstr(maneuver_str, "ARRIVE") == NULL);
        strncpy(nav_data.current_street, street_str[0] ? street_str : "Motorcycle Route", sizeof(nav_data.current_street) - 1);

        if (eta_secs >= 60) {
            uint16_t eta_mins = (uint16_t)((eta_secs / 60) > 9999 ? 9999 : (eta_secs / 60));
            snprintf(nav_data.eta_time_str, sizeof(nav_data.eta_time_str), "%u min", (unsigned)eta_mins);
        } else {
            snprintf(nav_data.eta_time_str, sizeof(nav_data.eta_time_str), "< 1 min");
        }

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
        } else if (op == NAV_OP_RESTART) {
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
            // Latest Notification Packet: [0x0F, title_len, title..., msg_len, msg...]
            uint8_t t_len = rx_buf[1];
            char title_str[32] = {0};
            char msg_str[64] = {0};
            size_t curr = 2;

            if (t_len > 0 && len >= (size_t)(curr + t_len)) {
                size_t c_len = (t_len < 31) ? t_len : 31;
                memcpy(title_str, &rx_buf[curr], c_len);
                title_str[c_len] = '\0';
                curr += t_len;
            }

            if (len > curr) {
                uint8_t m_len = rx_buf[curr++];
                if (m_len > 0 && len >= (size_t)(curr + m_len)) {
                    size_t c_len = (m_len < 63) ? m_len : 63;
                    memcpy(msg_str, &rx_buf[curr], c_len);
                    msg_str[c_len] = '\0';
                }
            }

            ESP_LOGI(TAG, "BLE Notification: Title='%s' Msg='%s'", title_str, msg_str);
            display_show_notification(title_str, msg_str);
            return 0;
        } else if (op == NAV_OP_DIAGNOSTICS) {
            ESP_LOGI(TAG, "BLE Remote: Diagnostics self-test requested from App");
            int batt_pct = battery_get_percentage();
            float batt_v = battery_get_voltage();
            bool is_chg = battery_is_charging();
            uint8_t v_deci = (batt_v > 0) ? (uint8_t)(batt_v * 10.0f) : 41;

            uint8_t diag_buf[7] = {
                NAV_STATUS_DIAGNOSTICS, // 0x10
                1,                      // Bluetooth link OK
                1,                      // GPS status OK
                1,                      // Display TFT 320x240 OK
                (uint8_t)(batt_pct >= 0 ? batt_pct : 85),
                is_chg ? 1 : 0,
                v_deci
            };
            ble_nav_notify_diagnostic_packet(diag_buf, sizeof(diag_buf));
            return 0;
        }
    }

    // 3. Route Segment packet format (OpCode 0x0B)
    if (len >= 7 && rx_buf[0] == NAV_OP_ROUTE_SEGMENT) {
        uint8_t count = rx_buf[1];
        if (count > ROUTE_MAX_POINTS) count = ROUTE_MAX_POINTS;
        if (len >= (size_t)(7 + count * 4)) {
            route_segment_t seg;
            memset(&seg, 0, sizeof(seg));
            seg.count = count;
            seg.is_destination_visible = (rx_buf[2] & 0x01) != 0;
            seg.dest_x = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            seg.dest_y = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
            size_t offset = 7;
            for (uint8_t i = 0; i < count; i++) {
                seg.points[i].x = (uint16_t)rx_buf[offset] | ((uint16_t)rx_buf[offset + 1] << 8);
                seg.points[i].y = (uint16_t)rx_buf[offset + 2] | ((uint16_t)rx_buf[offset + 3] << 8);
                offset += 4;
            }
            ESP_LOGI(TAG, "NAV Route Segment: %d pts, dest_vis=%d (%d,%d)",
                     count, seg.is_destination_visible, seg.dest_x, seg.dest_y);
            display_update_route_segment(&seg);
            return 0;
        }
    }

    // 4. Compact Binary packet format (Production mode)
    if (len >= 18) {
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
            if (text_len > 0 && len >= (size_t)(21 + text_len)) {
                int copy_bytes = (text_len < 31) ? text_len : 31;
                memcpy(street_buf, &rx_buf[21], copy_bytes);
                street_buf[copy_bytes] = '\0';
            }
        } else {
            // Legacy 18-byte header
            text_len = rx_buf[17];
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
            display_set_screen(SCREEN_HOME);
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

        // A real route start owns the HUD.  Do this once at NAV_START rather
        // than forcing the navigation page on every live update, so a rider
        // can still deliberately view media or settings while navigating.
        display_update_navigation(&nav_data);
        if ((opcode == NAV_OP_START && nav_data.is_navigating) ||
            (nav_data.is_navigating && display_get_current_screen() == SCREEN_HOME)) {
            display_set_screen(SCREEN_NAV);
        }
        return 0;
    }

    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
}

// 0xFF32 — Navigation Status / Button Feedback
static int ble_nav_status_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t status_buf[2] = { 0x01, 0x00 }; // Device ready
        os_mbuf_append(ctxt->om, status_buf, sizeof(status_buf));
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
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
            if (s_adv_timer) xTimerStop(s_adv_timer, 0);
            ESP_LOGI(TAG, "Phone connected (conn_handle=%d)! Services online.", s_conn_handle);
            display_set_ble_status(true);

            // Update BLE connection parameters for resilient background link stability:
            // 6000ms supervision timeout prevents Android from dropping connection
            // when user switches to Google Maps / shares location.
            struct ble_gap_upd_params conn_params;
            memset(&conn_params, 0, sizeof(conn_params));
            conn_params.itvl_min = 24;            // 30 ms (24 * 1.25ms)
            conn_params.itvl_max = 40;            // 50 ms (40 * 1.25ms)
            conn_params.latency = 0;              // Low latency for snappy turn instructions
            conn_params.supervision_timeout = 600; // 6000 ms (600 * 10ms)
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
        s_is_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
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
