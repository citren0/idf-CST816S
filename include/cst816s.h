
#ifndef CST816S_H_
#define CST816S_H_

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

// Pin definition.
#define CST_SCL_GPIO        GPIO_NUM_10
#define CST_SDA_GPIO        GPIO_NUM_11
#define CST_RST_GPIO        GPIO_NUM_13
#define CST_INT_GPIO        GPIO_NUM_14

// I2C Definition.
#define CST_I2C_ADDR        0x15
#define CST_I2C_FREQ        40000
#define CST_I2C_TIMEOUT     100

// Registers.
#define CST_LOWER_Y         ((uint8_t)0x06)
#define CST_UPPER_Y         ((uint8_t)0x05)
#define CST_LOWER_X         ((uint8_t)0x04)
#define CST_UPPER_X         ((uint8_t)0x03)
#define CST_FINGER_NUM      ((uint8_t)0x02)

#define LOWER_4_BIT_MASK    0x0f

struct touch
{
    uint16_t x;
    uint16_t y;
};

esp_err_t init_cst_interrupt(void);
esp_err_t init_cst(i2c_master_bus_handle_t bus_handle);
uint16_t get_y_coordinate(void);
uint16_t get_x_coordinate(void);
void get_touch(uint8_t * out_is_touched, struct touch * out_touch);
uint8_t is_touched(void);

#endif // CST816S_H_