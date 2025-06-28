#include "i2c_slave.h"
#include "radar_config.h"
#include "radar_processing.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "I2C_SLAVE";
static uint8_t last_register = 0;

static size_t prepare_tx_data(uint8_t reg, uint8_t *tx_buffer) {
    radar_config_t *config = radar_config_get();
    size_t tx_length = 1;

    switch (reg) {
        // --- Existing cases ---
        case REG_TEST_MODE_STATUS: tx_buffer[0] = I2C_TEST_MODE_ENABLED; break;
        case REG_MOTION_DETECTED: tx_buffer[0] = radar_config_get_motion_detected_safe() ? 1 : 0; break;
        case REG_PRESENCE_CONFIRMED: tx_buffer[0] = radar_config_get_presence_confirmed_safe() ? 1 : 0; break;
        case REG_SYSTEM_STATUS: tx_buffer[0] = 0x01; break;
        case REG_FRAME_DELAY_MS: memcpy(tx_buffer, &config->frame_delay_ms, sizeof(uint32_t)); tx_length = sizeof(uint32_t); break;
        case REG_BACKGROUND_ALPHA: memcpy(tx_buffer, &config->background_alpha, sizeof(float)); tx_length = sizeof(float); break;
        case REG_CFAR_NEAR_BIAS_DB: memcpy(tx_buffer, &config->near_range_bias_db, sizeof(float)); tx_length = sizeof(float); break;
        case REG_CFAR_FAR_BIAS_DB: memcpy(tx_buffer, &config->far_range_bias_db, sizeof(float)); tx_length = sizeof(float); break;
        case REG_CFAR_GUARDS: tx_buffer[0] = config->presence_cfar_guards; break;
        case REG_CFAR_REFS: tx_buffer[0] = config->presence_cfar_refs; break;
        case REG_HISTORY_LEN: tx_buffer[0] = config->history_len; break;
        case REG_MIN_DETECTIONS: tx_buffer[0] = config->min_detections_in_history; break;
        case REG_MAX_RANGE_DIFF_M: memcpy(tx_buffer, &config->max_range_diff_m, sizeof(float)); tx_length = sizeof(float); break;

        // --- NEW: Handle UART Plotting Read ---
        case REG_UART_PLOTTING:
            tx_buffer[0] = config->enable_uart_plotting ? 1 : 0;
            break;

        default: tx_buffer[0] = 0xFF; break; // Error: Unknown register
    }
    return tx_length;
}

static void process_write(uint8_t reg, const uint8_t *data, size_t len) {
    radar_config_t *config = radar_config_get();
    if (len == 0) return; // No data to process

    switch (reg) {
        // --- Existing cases ---
        case REG_FRAME_DELAY_MS: if (len >= 4) { uint32_t val; memcpy(&val, data, 4); radar_config_set_frame_delay(val); } break;
        case REG_BACKGROUND_ALPHA: if (len >= 4) { float val; memcpy(&val, data, 4); radar_config_set_presence_params(val, config->recalibration_interval_s); } break;
        case REG_RECALIBRATE_NOW: if(I2C_TEST_MODE_ENABLED == 0) radar_processing_recalibrate_now(); break;
        case REG_CFAR_NEAR_BIAS_DB: if (len >= 4) { float val; memcpy(&val, data, 4); radar_config_set_cfar_params(val, config->far_range_bias_db, config->presence_cfar_guards, config->presence_cfar_refs); } break;
        case REG_CFAR_FAR_BIAS_DB: if (len >= 4) { float val; memcpy(&val, data, 4); radar_config_set_cfar_params(config->near_range_bias_db, val, config->presence_cfar_guards, config->presence_cfar_refs); } break;
        case REG_CFAR_GUARDS: radar_config_set_cfar_params(config->near_range_bias_db, config->far_range_bias_db, data[0], config->presence_cfar_refs); break;
        case REG_CFAR_REFS: radar_config_set_cfar_params(config->near_range_bias_db, config->far_range_bias_db, config->presence_cfar_guards, data[0]); break;
        case REG_HISTORY_LEN: radar_config_set_temporal_filter_params(data[0], config->min_detections_in_history, config->max_range_diff_m); break;
        case REG_MIN_DETECTIONS: radar_config_set_temporal_filter_params(config->history_len, data[0], config->max_range_diff_m); break;
        case REG_MAX_RANGE_DIFF_M: if (len >= 4) { float val; memcpy(&val, data, 4); radar_config_set_temporal_filter_params(config->history_len, config->min_detections_in_history, val); } break;
        
        // --- NEW: Handle UART Plotting Write ---
        case REG_UART_PLOTTING:
            radar_config_set_uart_plotting(data[0] != 0);
            break;
    }
}

// --- i2c_slave_task, i2c_slave_init, i2c_slave_task_create are unchanged ---
void i2c_slave_task(void *pvParameters) {
    uint8_t rx_buffer[I2C_SLAVE_RX_BUF_LEN];
    uint8_t tx_buffer[I2C_SLAVE_TX_BUF_LEN];
    size_t tx_length = 1;
    ESP_LOGI(TAG, "I2C slave task started.");
    while (1) {
        int rx_len = i2c_slave_read_buffer(I2C_SLAVE_PORT, rx_buffer, I2C_SLAVE_RX_BUF_LEN, pdMS_TO_TICKS(100));
        if (rx_len > 0) {
            last_register = rx_buffer[0];
            ESP_LOGD(TAG, "I2C master interaction: Reg=0x%02X, Len=%d", last_register, rx_len);
            if (rx_len > 1) { process_write(last_register, rx_buffer + 1, rx_len - 1); }
            tx_length = prepare_tx_data(last_register, tx_buffer);
            i2c_slave_write_buffer(I2C_SLAVE_PORT, tx_buffer, tx_length, pdMS_TO_TICKS(5));
        }
    }
}

esp_err_t i2c_slave_init(void) {
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_PIN,
        .scl_io_num = I2C_SLAVE_SCL_PIN,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_SLAVE_PORT, &conf_slave);
    return i2c_driver_install(I2C_SLAVE_PORT, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void i2c_slave_task_create(void) {
    if (i2c_slave_init() != ESP_OK) { ESP_LOGE(TAG, "Failed to initialize I2C slave"); return; }
    xTaskCreate(i2c_slave_task, "i2c_slave", 4096, NULL, 6, NULL);
}
