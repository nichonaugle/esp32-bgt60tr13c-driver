#include "radar_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "RADAR_CONFIG";

// Global configuration instance
static radar_config_t g_radar_config;
static SemaphoreHandle_t config_mutex = NULL;
static bool interrupt_pin_initialized = false;
static bool last_motion_state = false;

void radar_config_init(void) {
    // Initialize with default values (matching original defines)
    g_radar_config.antenna_index = 1;
    g_radar_config.useful_range_m = 5.0f;
    g_radar_config.num_guard_cells = 4;
    g_radar_config.num_ref_cells = 10;
    g_radar_config.cfar_bias_db = 13.0f;
    g_radar_config.enable_uart_plotting = false;
    g_radar_config.frame_delay_ms = 500;
    
    // Initialize test mode settings
    g_radar_config.motion_detected = false;
    
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

// Test mode functions
void radar_config_set_motion_detected(bool detected) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    
    bool previous_state = g_radar_config.motion_detected;
    g_radar_config.motion_detected = detected;
    
    xSemaphoreGive(config_mutex);
    
    // Trigger interrupt to I2C master if state changed
    if (detected != previous_state) {
        radar_config_trigger_motion_interrupt(detected);
    }
}

bool radar_config_get_motion_detected(void) {
    return g_radar_config.motion_detected;
}

// I2C master interrupt functions
esp_err_t radar_config_init_interrupt_pin(void) {
    esp_err_t ret;
    
    // Configure GPIO for output to I2C master
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_IRQ_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C master interrupt pin GPIO %d: %s", 
                 I2C_MASTER_IRQ_PIN, esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize pin to LOW (no motion detected initially)
    gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, 0);
    
    interrupt_pin_initialized = true;
    last_motion_state = false;
    
    ESP_LOGI(TAG, "I2C master interrupt pin GPIO %d initialized", I2C_MASTER_IRQ_PIN);
    return ESP_OK;
}

void radar_config_trigger_motion_interrupt(bool motion_detected) {
    if (!interrupt_pin_initialized) {
        ESP_LOGW(TAG, "Interrupt pin not initialized, skipping interrupt trigger");
        return;
    }
    
    // Only trigger interrupt on state changes
    if (motion_detected != last_motion_state) {
        // Set GPIO level based on motion detection
        // HIGH = motion detected, LOW = no motion
        gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, motion_detected ? 1 : 0);
        
        ESP_LOGI(TAG, "Motion interrupt triggered: GPIO %d set to %s (motion %s)", 
                 I2C_MASTER_IRQ_PIN,
                 motion_detected ? "HIGH" : "LOW",
                 motion_detected ? "DETECTED" : "NOT DETECTED");
        
        last_motion_state = motion_detected;
    }
}

void radar_config_update_interrupt_pin(void) {
    if (interrupt_pin_initialized) {
        // Ensure GPIO reflects current motion state
        gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, g_radar_config.motion_detected ? 1 : 0);
    }
}

// Add this function
bool radar_config_get_motion_detected_safe(void) {
    bool detected;
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
        detected = g_radar_config.motion_detected;
        xSemaphoreGive(config_mutex);
    } else {
        detected = g_radar_config.motion_detected;
    }
    return detected;
}