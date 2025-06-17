#include "i2c_slave.h"
#include "radar_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "I2C_SLAVE";

static uint8_t last_register = 0;
static uint8_t tx_buffer[8];
static size_t tx_length = 1;

static void prepare_tx_data(uint8_t reg) {
    radar_config_t *config = radar_config_get();
    
    switch (reg) {
        case REG_ANTENNA_INDEX:
            tx_buffer[0] = config->antenna_index;
            tx_length = 1;
            break;
        case REG_USEFUL_RANGE:
            memcpy(tx_buffer, &config->useful_range_m, sizeof(float));
            tx_length = 4;
            break;
        case REG_GUARD_CELLS:
            tx_buffer[0] = config->num_guard_cells;
            tx_length = 1;
            break;
        case REG_REF_CELLS:
            tx_buffer[0] = config->num_ref_cells;
            tx_length = 1;
            break;
        case REG_CFAR_BIAS:
            memcpy(tx_buffer, &config->cfar_bias_db, sizeof(float));
            tx_length = 4;
            break;
        case REG_UART_PLOTTING:
            tx_buffer[0] = config->enable_uart_plotting ? 1 : 0;
            tx_length = 1;
            break;
        case REG_FRAME_DELAY:
            memcpy(tx_buffer, &config->frame_delay_ms, sizeof(uint32_t));
            tx_length = 4;
            break;
        case REG_SYSTEM_STATUS:
            tx_buffer[0] = 0x01; // Running
            tx_length = 1;
            break;
        case REG_TEST_MODE:
            tx_buffer[0] = I2C_TEST_MODE_ENABLED ? 1 : 0;
            tx_length = 1;
            break;
        case REG_MOTION_DETECTED:
            // We use our thread-safe function to get the REAL motion state
            tx_buffer[0] = radar_config_get_motion_detected_safe() ? 1 : 0;
            tx_length = 1;
            break;
        default:
            tx_buffer[0] = 0xFF; // Error
            tx_length = 1;
            break;
    }
}

static void process_write(uint8_t reg, const uint8_t *data, size_t len) {
    radar_config_t *config = radar_config_get();
    
    switch (reg) {
        case REG_ANTENNA_INDEX:
            if (len >= 1) radar_config_set_antenna_index(data[0]);
            break;
        case REG_USEFUL_RANGE:
            if (len >= 4) {
                float range;
                memcpy(&range, data, sizeof(float));
                radar_config_set_useful_range(range);
            }
            break;
        case REG_GUARD_CELLS:
            if (len >= 1) radar_config_set_cfar_params(data[0], config->num_ref_cells, config->cfar_bias_db);
            break;
        case REG_REF_CELLS:
            if (len >= 1) radar_config_set_cfar_params(config->num_guard_cells, data[0], config->cfar_bias_db);
            break;
        case REG_CFAR_BIAS:
            if (len >= 4) {
                float bias;
                memcpy(&bias, data, sizeof(float));
                radar_config_set_cfar_params(config->num_guard_cells, config->num_ref_cells, bias);
            }
            break;
        case REG_UART_PLOTTING:
            if (len >= 1) radar_config_set_uart_plotting(data[0] != 0);
            break;
        case REG_FRAME_DELAY:
            if (len >= 4) {
                uint32_t delay;
                memcpy(&delay, data, sizeof(uint32_t));
                radar_config_set_frame_delay(delay);
            }
            break;
    }
}

void i2c_slave_task(void *pvParameters) {
    uint8_t rx_buffer[128];
    int rx_len = 0;

    ESP_LOGI(TAG, "Final I2C slave task started.");

    while (1) {
        rx_len = i2c_slave_read_buffer(I2C_SLAVE_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(100));

        if (rx_len > 0) {
            last_register = rx_buffer[0];
            ESP_LOGD(TAG, "I2C master interaction: Reg=0x%02X, Len=%d", last_register, rx_len);

            if (rx_len > 1) { // Master is WRITING
                process_write(last_register, rx_buffer + 1, rx_len - 1);
            } else { // Master is setting register for a READ
                prepare_tx_data(last_register);
                i2c_slave_write_buffer(I2C_SLAVE_PORT, tx_buffer, tx_length, 0);
            }
        }
    }
}

esp_err_t i2c_slave_init(void) {
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_PIN,
        .scl_io_num = I2C_SLAVE_SCL_PIN,
        .slave.slave_addr = I2C_SLAVE_ADDR,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_SLAVE_PORT, &conf_slave);
    return i2c_driver_install(I2C_SLAVE_PORT, conf_slave.mode, 128, 128, 0);
}

void i2c_slave_task_create(void) {
    if (i2c_slave_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C slave");
        return;
    }
    // Use standard xTaskCreate, but with a high priority to ensure responsiveness.
    xTaskCreate(i2c_slave_task, "i2c_slave", 4096, NULL, 6, NULL);
}