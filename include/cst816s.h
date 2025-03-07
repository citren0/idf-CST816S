
#ifndef CST816S_H_
#define CST816S_H_

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CST_I2C_ADDR    0x15
#define CST_I2C_FREQ    40000
#define CST_I2C_TIMEOUT 1000

// Registers
#define CST_LOWER_Y     ((uint8_t)0x06)
#define CST_UPPER_Y     ((uint8_t)0x05)
#define CST_LOWER_X     ((uint8_t)0x04)
#define CST_UPPER_X     ((uint8_t)0x03)

esp_err_t init_interrupt(int pin);
esp_err_t init_cst(int scl, int sda);
uint16_t get_y_coordinate(void);
uint16_t get_x_coordinate(void);


#endif // CST816S_H_