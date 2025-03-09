
#include "cst816s.h"


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

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
            if (xSemaphoreTake(semaphore, 5 / portTICK_PERIOD_MS))
            {
                if ((last_is_touched = is_touched()) == 1)
                {
                    last_touch.x = get_x_coordinate();
                    last_touch.y = get_y_coordinate();
                }

                xSemaphoreGive(semaphore);
            }
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
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


esp_err_t init_cst(void)
{
    // Reset chip.
    gpio_set_direction(CST_RST_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(CST_RST_GPIO, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(CST_RST_GPIO, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    gpio_set_level(CST_RST_GPIO, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Init I2C
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CST_SDA_GPIO,
        .scl_io_num = CST_SCL_GPIO,
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

    return (((data[1] & LOWER_4_BIT_MASK) << 8) | (data[0] << 0));
}


uint16_t get_x_coordinate(void)
{
    uint8_t data[2];

    uint8_t lower_buf[1] = { CST_LOWER_X };
    uint8_t upper_buf[1] = { CST_UPPER_X };

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, lower_buf, 1, &data[0], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, upper_buf, 1, &data[1], 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));

    return (((data[1] & LOWER_4_BIT_MASK) << 8) | (data[0] << 0));
}


uint8_t is_touched(void)
{
    uint8_t data;

    uint8_t cmd[1] = { CST_FINGER_NUM };

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, cmd, 1, &data, 1, CST_I2C_TIMEOUT / portTICK_PERIOD_MS));

    return data;
}
