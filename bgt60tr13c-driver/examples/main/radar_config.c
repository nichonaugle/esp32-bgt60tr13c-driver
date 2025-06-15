#include "radar_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Global configuration instance
static radar_config_t g_radar_config;
static SemaphoreHandle_t config_mutex = NULL;

void radar_config_init(void) {
    // Initialize with default values (matching original defines)
    g_radar_config.antenna_index = 1;
    g_radar_config.useful_range_m = 5.0f;
    g_radar_config.num_guard_cells = 4;
    g_radar_config.num_ref_cells = 10;
    g_radar_config.cfar_bias_db = 13.0f;
    g_radar_config.enable_uart_plotting = false;
    g_radar_config.frame_delay_ms = 500;
    
    // Create mutex for thread-safe access
    config_mutex = xSemaphoreCreateMutex();
}

radar_config_t* radar_config_get(void) {
    return &g_radar_config;
}

void radar_config_set_antenna_index(uint8_t index) {
    if (index < NUM_RX_ANTENNAS) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
        g_radar_config.antenna_index = index;
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_useful_range(float range_m) {
    if (range_m > 0 && range_m <= R_MAX_M) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
        g_radar_config.useful_range_m = range_m;
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_cfar_params(uint8_t guard_cells, uint8_t ref_cells, float bias_db) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.num_guard_cells = guard_cells;
    g_radar_config.num_ref_cells = ref_cells;
    g_radar_config.cfar_bias_db = bias_db;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_uart_plotting(bool enable) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.enable_uart_plotting = enable;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_frame_delay(uint32_t delay_ms) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.frame_delay_ms = delay_ms;
    xSemaphoreGive(config_mutex);
}