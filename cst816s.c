
#include "cst816s.h"


static i2c_master_dev_handle_t dev_handle;

static QueueHandle_t events = NULL;
static SemaphoreHandle_t semaphore = NULL;

static struct touch last_touch = { 0 };
static uint8_t last_is_touched = 0;


static void IRAM_ATTR isr_handler(void * args)
{
    uint8_t item = 1;
    xQueueSendFromISR(events, &item, NULL);
}


void get_touch(uint8_t * out_is_touched, struct touch * out_touch)
{
    if (xSemaphoreTake(semaphore, 0))
    {
        *out_is_touched = last_is_touched;
        out_touch->x = last_touch.x;
        out_touch->y = last_touch.y;
        
        xSemaphoreGive(semaphore);
    }
    // Don't overwrite anything if we can't get the semaphore.
}


void check_touch_task(void * args)
{
    uint8_t receive;

    while (1)
    {
        if (xQueueReceive(events, &receive, 0))
        {
            if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(5)))
            {
                if ((last_is_touched = is_touched()) == 1)
                {
                    last_touch.x = get_x_coordinate();
                    last_touch.y = get_y_coordinate();
                }

                xSemaphoreGive(semaphore);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


esp_err_t init_cst_interrupt(void)
{
    events = xQueueCreate(1, sizeof(char));
    semaphore = xSemaphoreCreateMutex();

    xTaskCreate(check_touch_task, "CST816S_check_touch_task", 4096, NULL, 5, NULL);

    gpio_config_t int_config = {
        .pin_bit_mask = (1ULL << CST_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&int_config);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(CST_INT_GPIO, isr_handler, NULL);

    return ESP_OK;
}


esp_err_t init_cst(i2c_master_bus_handle_t bus_handle)
{
    // Reset chip.
    gpio_set_direction(CST_RST_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(CST_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CST_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(CST_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

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

    i2c_master_transmit_receive(dev_handle, lower_buf, sizeof(lower_buf), &data[0], 1, pdMS_TO_TICKS(CST_I2C_TIMEOUT));
    i2c_master_transmit_receive(dev_handle, upper_buf, sizeof(upper_buf), &data[1], 1, pdMS_TO_TICKS(CST_I2C_TIMEOUT));

    return (((data[1] & LOWER_4_BIT_MASK) << 8) | (data[0] << 0));
}


uint16_t get_x_coordinate(void)
{
    uint8_t data[2];

    uint8_t lower_buf[1] = { CST_LOWER_X };
    uint8_t upper_buf[1] = { CST_UPPER_X };

    i2c_master_transmit_receive(dev_handle, lower_buf, 1, &data[0], 1, pdMS_TO_TICKS(CST_I2C_TIMEOUT));
    i2c_master_transmit_receive(dev_handle, upper_buf, 1, &data[1], 1, pdMS_TO_TICKS(CST_I2C_TIMEOUT));

    return (((data[1] & LOWER_4_BIT_MASK) << 8) | (data[0] << 0));
}


uint8_t is_touched(void)
{
    uint8_t data;

    uint8_t cmd[1] = { CST_FINGER_NUM };

    i2c_master_transmit_receive(dev_handle, cmd, 1, &data, 1, pdMS_TO_TICKS(CST_I2C_TIMEOUT));

    return data;
}
