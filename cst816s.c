
#include "cst816s.h"


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


esp_err_t init_interrupt(int pin)
{
    // TODO
    return ESP_OK;
}


esp_err_t init_cst(int scl, int sda)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CST_I2C_ADDR,
        .scl_speed_hz = CST_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    return ESP_OK;
}


uint16_t get_y_coordinate(void)
{
    uint8_t data[2];

    uint8_t lower_buf[1] = { CST_LOWER_Y };
    uint8_t upper_buf[1] = { CST_UPPER_Y };

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, lower_buf, sizeof(lower_buf), &data[0], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, upper_buf, sizeof(upper_buf), &data[1], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));

    return ((data[1] << 8) | (data[0] << 0));
}


uint16_t get_x_coordinate(void)
{
    uint8_t data[2];

    uint8_t lower_buf[1] = { CST_LOWER_X };
    uint8_t upper_buf[1] = { CST_UPPER_X };

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, lower_buf, 1, &data[0], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, upper_buf, 1, &data[1], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));

    return ((data[1] << 8) | (data[0] << 0));
}
