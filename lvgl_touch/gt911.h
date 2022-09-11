
/**
 * @file gt911.h
 */

#ifndef __GT911_H
#define __GT911_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif



#define GT911_RST 4
#define GT911_INT 5

#define CTP_GT911_SLAVE_ADDRESS        0x5D
#define CTP_GT911_INT_PIN              17
#define CTP_GT911_RST_PIN              5

#define GOODIX_CONTACT_SIZE            8
#define GOODIX_MAX_CONTACTS            10

#define GT_REG_DATA                    0x8140
#define GOODIX_READ_COORD_ADDR         0x814E

/* Maximum border values of the touchscreen pad that the chip can handle */
#define  GT911_MAX_WIDTH              ((uint16_t)480)
#define  GT911_MAX_HEIGHT             ((uint16_t)320)


typedef struct {
    bool inited;
} gt911_status_t;

struct GTInfo {
    // 0x8140-0x814A
    char productId[4];
    uint16_t fwId;
    uint16_t xResolution;
    uint16_t yResolution;
    uint8_t vendorId;
};

struct GTPoint {
    // 0x814F-0x8156, ... 0x8176 (5 points)
    uint8_t trackId;
    uint16_t x;
    uint16_t y;
    uint16_t area;
    uint8_t reserved;
} __attribute__((packed));


/**
  * @brief  Initialize for GT911 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of GT911).
  * @retval None
  */
void gt911_init(uint16_t dev_addr);


/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool gt911_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#ifdef __cplusplus
}
#endif
#endif /* __GT911_H */
