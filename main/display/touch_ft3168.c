#include "touch.h"
#include "waveshare143.h"
#include "round_panel.h"
#include "board_decode.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <string.h>
esp_err_t touch_init(void) {
    esp_err_t e=waveshare143_i2c_init();
    if(e==ESP_OK)e=waveshare143_write(0x38,0x00,0);
    ESP_LOGI("FT3168","Touch 0x38 SDA=%d SCL=%d: %s",WS143_I2C_SDA,WS143_I2C_SCL,esp_err_to_name(e));
    return e;
}
bool touch_get_point(touch_point_t *p) {
    if(!p)return false;
    memset(p,0,sizeof(*p));
    uint8_t data[5];int x,y;
    if(waveshare143_read(0x38,2,data,5)!=ESP_OK || !ft3168_decode(data,&x,&y))return false;
#if CONFIG_JR_TOUCH_SWAP_XY
    int tmp=x;x=y;y=tmp;
#endif
#if CONFIG_JR_TOUCH_MIRROR_X
    x=465-x;
#endif
#if CONFIG_JR_TOUCH_MIRROR_Y
    y=465-y;
#endif
    // Preserve raw contact outside the inscribed auxiliary canvas for wake-only.
    p->pressure=1;p->is_pressed=true;
    return round_panel_touch(x,y,&p->x,&p->y);
}
