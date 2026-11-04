#include "sdkconfig.h"
#include "device_power.h"
#include "ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#if CONFIG_JUSTRIDE_ROUND_AMOLED
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t calibration;
static portMUX_TYPE power_lock=portMUX_INITIALIZER_UNLOCKED;
static device_power_t latest={.percent=255};
device_power_t device_power_snapshot(void) {
    portENTER_CRITICAL(&power_lock);
    device_power_t p=latest;
    portEXIT_CRITICAL(&power_lock);return p;
}
static void power_task(void *unused) {
    unsigned ticks=0;
    for(;;) {
        int sum=0,raw,minimum=4096,maximum=0;
        bool ok=true;
        for(int i=0;i<16;i++) {
            if(adc_oneshot_read(adc,ADC_CHANNEL_3,&raw)!=ESP_OK){ok=false;break;}
            sum+=raw;if(raw<minimum)minimum=raw;if(raw>maximum)maximum=raw;
        }
        int mv=0;
        if(ok)ok=adc_cali_raw_to_voltage(calibration,(sum-minimum-maximum)/14,&mv)==ESP_OK;
        device_power_t p=power_measurement(ok?mv:0,CONFIG_JR_POWER_GAIN_PERMILLE,
            device_power_snapshot().source);
        portENTER_CRITICAL(&power_lock);latest=p;portEXIT_CRITICAL(&power_lock);
        if(ticks++%5==0)device_power_publish();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void device_power_start(void) {
    adc_oneshot_unit_init_cfg_t unit={.unit_id=ADC_UNIT_1};
    adc_oneshot_chan_cfg_t channel={.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    adc_cali_curve_fitting_config_t config={.unit_id=ADC_UNIT_1,.chan=ADC_CHANNEL_3,
        .atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    esp_err_t e=adc_oneshot_new_unit(&unit,&adc);
    if(e==ESP_OK)e=adc_oneshot_config_channel(adc,ADC_CHANNEL_3,&channel);
    if(e==ESP_OK)e=adc_cali_create_scheme_curve_fitting(&config,&calibration);
    if(e!=ESP_OK){ESP_LOGW("POWER","Calibrated VCC reading unavailable: %s",esp_err_to_name(e));return;}
    if(xTaskCreate(power_task,"device_power",3072,NULL,1,NULL)!=pdPASS) {
        ESP_LOGW("POWER","Power sampling task unavailable");return;
    }
    ESP_LOGI("POWER","GPIO4 = VCC / 3; charge status unavailable (STAT is LED-only)");
}
#else
device_power_t device_power_snapshot(void){return (device_power_t){.percent=255};}
void device_power_start(void){}
#endif
void device_power_publish(void) {
    uint8_t packet[10];power_encode(device_power_snapshot(),packet);
    ble_nav_notify_diagnostic_packet(packet,sizeof packet);
}
