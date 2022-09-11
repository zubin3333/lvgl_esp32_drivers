/*
* Copyright © 2020 Wolfgang Christl

* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the “Software”), to deal in the Software 
* without restriction, including without limitation the rights to use, copy, modify, merge, 
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
* to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
* SOFTWARE.
*/

#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "gt911.h"
#include "tp_i2c.h"
#include "../lvgl_i2c_conf.h"

#define TAG "GT911"


gt911_status_t gt911_status;
uint8_t current_dev_addr;       // set during init

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 0x1                                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                     /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                                           /*!< I2C ack value */
#define NACK_VAL 0x1                                          /*!< I2C nack value */
esp_err_t i2c_master_read_slave_reg_16bit(uint8_t slave_addr, uint16_t reg_addr, uint8_t *data_rd, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_master_write_slave_reg_16bit(uint8_t slave_addr, uint16_t reg_addr, uint8_t *data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
  * @brief  Initialize for GT911 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of FT6X36).
  * @retval None
  */
void gt911_init(uint16_t dev_addr) {
    if (!gt911_status.inited) {

/* I2C master is initialized before calling this function */
#if 0
        esp_err_t code = i2c_master_init();
#else
        esp_err_t code = ESP_OK;
#endif

        if (code != ESP_OK) {
            gt911_status.inited = false;
            ESP_LOGE(TAG, "Error during I2C init %s", esp_err_to_name(code));
        } else {
            gt911_status.inited = true;

            // Reset Chip
            esp_rom_gpio_pad_select_gpio(CTP_GT911_INT_PIN);
            esp_rom_gpio_pad_select_gpio(CTP_GT911_RST_PIN);
            gpio_set_direction(CTP_GT911_INT_PIN, GPIO_MODE_OUTPUT);
            gpio_set_direction(CTP_GT911_RST_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(CTP_GT911_INT_PIN, 0);
            gpio_set_level(CTP_GT911_RST_PIN, 0);
            vTaskDelay(11 / portTICK_PERIOD_MS);
            gpio_set_level(CTP_GT911_INT_PIN, 0);
            vTaskDelay(110 / portTICK_PERIOD_MS);
            gpio_set_level(CTP_GT911_RST_PIN, 1);
            gpio_set_pull_mode(CTP_GT911_RST_PIN, GPIO_PULLUP_ENABLE);
            gpio_set_direction(CTP_GT911_RST_PIN, GPIO_MODE_INPUT);
            vTaskDelay(6 / portTICK_PERIOD_MS);
            gpio_set_level(CTP_GT911_INT_PIN, 0);
            vTaskDelay(51 / portTICK_PERIOD_MS);
            gpio_set_direction(CTP_GT911_INT_PIN, GPIO_MODE_INPUT);
            vTaskDelay(100 / portTICK_PERIOD_MS);


            current_dev_addr = dev_addr;
            struct GTInfo info;
            esp_err_t ret;
            ESP_LOGI(TAG, "Found touch panel controller");
            if ((ret = i2c_master_read_slave_reg_16bit(dev_addr, GT_REG_DATA, (uint8_t *) &info, sizeof(info)) != ESP_OK))
                ESP_LOGE(TAG, "Error reading from device: %s",
                         esp_err_to_name(ret));    // Only show error the first time
            ESP_LOGI(TAG, "\tFirmware ID: 0x%02x", info.fwId);
            ESP_LOGI(TAG, "\tX-Resolution: %d", info.xResolution);
            ESP_LOGI(TAG, "\tY-Resolution: %d", info.yResolution);
            ESP_LOGI(TAG, "\tVendor ID: 0x%02x", info.vendorId);
        }
    }
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool gt911_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint8_t zpoints[GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE];
    static int16_t last_x = 0;    // 12bit pixel value
    static int16_t last_y = 0;    // 12bit pixel value

    uint8_t regState[1];
    i2c_master_read_slave_reg_16bit(current_dev_addr, GOODIX_READ_COORD_ADDR, regState, 1);
    int touch_num = regState[0] & 0x0f;
    ESP_LOGI(TAG, "\tnum touch: %d", touch_num);
    if (touch_num != 1) {    // ignore no touch & multi touch
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;
        u_int8_t write_byte = 0; 
        i2c_master_write_slave_reg_16bit(CTP_GT911_SLAVE_ADDRESS, GOODIX_READ_COORD_ADDR, &write_byte, 1);
        return false;
    }

    // Read X, Y values
    esp_err_t ret = i2c_master_read_slave_reg_16bit(current_dev_addr, GOODIX_READ_COORD_ADDR + 1, zpoints, GOODIX_CONTACT_SIZE * (touch_num));
    u_int8_t write_byte = 0; 
    i2c_master_write_slave_reg_16bit(CTP_GT911_SLAVE_ADDRESS, GOODIX_READ_COORD_ADDR, &write_byte, 1);
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Error getting X coordinates: %s", esp_err_to_name(ret));
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;   // no touch detected
        return false;
    }
    struct GTPoint * points = (struct GTPoint *)zpoints;
    
    last_x = points[0].x;
    last_y = points[0].y;




    // // Read X value
    // i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();

    // i2c_master_start(i2c_cmd);
    // i2c_master_write_byte(i2c_cmd, (current_dev_addr << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(i2c_cmd, FT6X36_P1_XH_REG, I2C_MASTER_ACK);

    // i2c_master_start(i2c_cmd);
    // i2c_master_write_byte(i2c_cmd, (current_dev_addr << 1) | I2C_MASTER_READ, true);

    // i2c_master_read_byte(i2c_cmd, &data_xy[0], I2C_MASTER_ACK);     // reads FT6X36_P1_XH_REG
    // i2c_master_read_byte(i2c_cmd, &data_xy[1], I2C_MASTER_NACK);    // reads FT6X36_P1_XL_REG
    // i2c_master_stop(i2c_cmd);
    // esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, i2c_cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(i2c_cmd);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error getting X coordinates: %s", esp_err_to_name(ret));
    //     data->point.x = last_x;
    //     data->point.y = last_y;
    //     data->state = LV_INDEV_STATE_REL;   // no touch detected
    //     return false;
    // }

    // // Read Y value
    // i2c_cmd = i2c_cmd_link_create();

    // i2c_master_start(i2c_cmd);
    // i2c_master_write_byte(i2c_cmd, (current_dev_addr << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(i2c_cmd, FT6X36_P1_YH_REG, I2C_MASTER_ACK);

    // i2c_master_start(i2c_cmd);
    // i2c_master_write_byte(i2c_cmd, (current_dev_addr << 1) | I2C_MASTER_READ, true);

    // i2c_master_read_byte(i2c_cmd, &data_xy[2], I2C_MASTER_ACK);     // reads FT6X36_P1_YH_REG
    // i2c_master_read_byte(i2c_cmd, &data_xy[3], I2C_MASTER_NACK);    // reads FT6X36_P1_YL_REG
    // i2c_master_stop(i2c_cmd);
    // ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, i2c_cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(i2c_cmd);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error getting Y coordinates: %s", esp_err_to_name(ret));
    //     data->point.x = last_x;
    //     data->point.y = last_y;
    //     data->state = LV_INDEV_STATE_REL;   // no touch detected
    //     return false;
    // }

    // last_x = ((data_xy[0] & FT6X36_MSB_MASK) << 8) | (data_xy[1] & FT6X36_LSB_MASK);
    // last_y = ((data_xy[2] & FT6X36_MSB_MASK) << 8) | (data_xy[3] & FT6X36_LSB_MASK);

#if CONFIG_LV_GT911_SWAPXY
    int16_t swap_buf = last_x;
    last_x = last_y;
    last_y = swap_buf;
#endif
#if CONFIG_LV_GT911_INVERT_X
    last_x =  LV_HOR_RES - last_x;
#endif
#if CONFIG_LV_GT911_INVERT_Y
    last_y = LV_VER_RES - last_y;
#endif
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_PR;
    ESP_LOGV(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    return false;
}
