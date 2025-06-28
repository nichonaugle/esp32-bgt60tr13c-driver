#include "radar_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "RADAR_CONFIG";

static radar_config_t g_radar_config;
static SemaphoreHandle_t config_mutex = NULL;
static bool interrupt_pin_initialized = false;
static volatile bool last_motion_state = false;

void radar_config_init(void) {
    config_mutex = xSemaphoreCreateMutex();

    g_radar_config.frame_delay_ms = 0;
    g_radar_config.background_alpha = 0.001f;
    g_radar_config.recalibration_interval_s = 60;
    g_radar_config.near_range_bias_db = 3.0f;
    g_radar_config.far_range_bias_db = 1.25f;
    g_radar_config.presence_cfar_guards = 5;
    g_radar_config.presence_cfar_refs = 4;
    g_radar_config.history_len = 3;
    g_radar_config.min_detections_in_history = 2;
    g_radar_config.max_range_diff_m = 2.0f;
    g_radar_config.motion_detected = false;
    g_radar_config.presence_confirmed = false;
    g_radar_config.enable_uart_plotting = true;
}

radar_config_t* radar_config_get(void) {
    return &g_radar_config;
}

// Setter for UART Plotting
void radar_config_set_uart_plotting(bool enable) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.enable_uart_plotting = enable;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
    ESP_LOGI(TAG, "UART plotting %s", enable ? "enabled" : "disabled");
}

// Other Setters
void radar_config_set_frame_delay(uint32_t delay_ms) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.frame_delay_ms = delay_ms;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_presence_params(float bg_alpha, uint32_t recal_interval_s) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.background_alpha = bg_alpha;
    g_radar_config.recalibration_interval_s = recal_interval_s;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_cfar_params(float near_bias, float far_bias, uint8_t guards, uint8_t refs) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.near_range_bias_db = near_bias;
    g_radar_config.far_range_bias_db = far_bias;
    g_radar_config.presence_cfar_guards = guards;
    g_radar_config.presence_cfar_refs = refs;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_temporal_filter_params(uint8_t hist_len, uint8_t min_dets, float max_range_diff) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.history_len = hist_len;
    g_radar_config.min_detections_in_history = min_dets;
    g_radar_config.max_range_diff_m = max_range_diff;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
}

// State Management
void radar_config_set_motion_detected(bool detected) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    g_radar_config.motion_detected = detected;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
}

void radar_config_set_presence_confirmed(bool confirmed) {
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    bool state_changed = (g_radar_config.presence_confirmed != confirmed);
    g_radar_config.presence_confirmed = confirmed;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }

    if (state_changed) {
        radar_config_trigger_motion_interrupt();
    }
}

bool radar_config_get_motion_detected_safe(void) {
    bool detected;
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    detected = g_radar_config.motion_detected;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
    return detected;
}

bool radar_config_get_presence_confirmed_safe(void) {
    bool confirmed;
    if (config_mutex != NULL) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
    }
    confirmed = g_radar_config.presence_confirmed;
    if (config_mutex != NULL) {
        xSemaphoreGive(config_mutex);
    }
    return confirmed;
}

// I2C MASTER INTERRUPT
esp_err_t radar_config_init_interrupt_pin(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_IRQ_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C master interrupt pin GPIO %d: %s",
                 I2C_MASTER_IRQ_PIN, esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, 0);
    interrupt_pin_initialized = true;
    last_motion_state = false;
    ESP_LOGI(TAG, "I2C master interrupt pin GPIO %d initialized", I2C_MASTER_IRQ_PIN);
    return ESP_OK;
}

void radar_config_trigger_motion_interrupt(void) {
    if (!interrupt_pin_initialized) {
        return;
    }
    bool current_state = radar_config_get_presence_confirmed_safe();
    if (current_state != last_motion_state) {
        gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, current_state ? 1 : 0);
        ESP_LOGI(TAG, "Presence Interrupt Triggered: GPIO %d set to %s",
                 I2C_MASTER_IRQ_PIN, current_state ? "HIGH" : "LOW");
        last_motion_state = current_state;
    }
}
