// radar_config.c

#include "radar_config.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <time.h>

static const char *TAG = "RADAR_CONFIG";

// Global configuration struct
static radar_config_t g_radar_config;
static SemaphoreHandle_t config_mutex = NULL;

// For occupancy state management
static time_t last_confirmed_presence_timestamp = 0;

// For I2C master interrupt pin
static bool interrupt_pin_initialized = false;
static volatile bool last_interrupt_pin_state = false;

// Helper to get monotonic time in seconds
static inline time_t get_time_s(void) {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return spec.tv_sec;
}

// Occupancy state management task
void occupancy_management_task(void *pvParameters) {
    ESP_LOGI(TAG, "Occupancy management task started.");
    while (1) {
        // Run this check every second
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (xSemaphoreTake(config_mutex, portMAX_DELAY) == pdTRUE) {
            bool confirmed = g_radar_config.presence_confirmed;
            bool current_occupancy = g_radar_config.occupancy_state;
            bool new_occupancy = current_occupancy;

            if (confirmed) {
                // If presence is confirmed, occupancy is active. Reset the timer.
                new_occupancy = true;
                last_confirmed_presence_timestamp = get_time_s();
            } else {
                // If presence is not confirmed, check if the off-delay has expired.
                if (current_occupancy) {
                    if ((get_time_s() - last_confirmed_presence_timestamp) > g_radar_config.occupancy_delay_s) {
                        new_occupancy = false;
                    }
                }
            }

            if (new_occupancy != current_occupancy) {
                g_radar_config.occupancy_state = new_occupancy;
                ESP_LOGI(TAG, "Occupancy state changed to: %s", new_occupancy ? "OCCUPIED" : "VACANT");

                // Trigger the physical interrupt pin if the state has changed
                if (interrupt_pin_initialized && new_occupancy != last_interrupt_pin_state) {
                    gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, new_occupancy ? 1 : 0);
                    ESP_LOGI(TAG, "Occupancy Interrupt: GPIO %d set to %s",
                             I2C_MASTER_IRQ_PIN, new_occupancy ? "HIGH" : "LOW");
                    last_interrupt_pin_state = new_occupancy;
                }
            }
            xSemaphoreGive(config_mutex);
        }
    }
}

void occupancy_management_task_create(void) {
    xTaskCreate(occupancy_management_task, "occupancy_task", 2048, NULL, 5, NULL);
}

void radar_config_init(void) {
    config_mutex = xSemaphoreCreateMutex();

    g_radar_config.frame_delay_ms = 0;
    g_radar_config.background_alpha = 0.01f;
    g_radar_config.recalibration_interval_s = 60;
    g_radar_config.near_range_bias_db = 3.15f;
    g_radar_config.far_range_bias_db = 2.25f;
    g_radar_config.presence_cfar_guards = 5;
    g_radar_config.presence_cfar_refs = 6;
    g_radar_config.history_len = 3;
    g_radar_config.min_detections_in_history = 3;
    g_radar_config.max_range_diff_m = 2.00f;
    g_radar_config.coherent_integration_factor = 4;
    g_radar_config.moving_avg_window_size = 7;
    g_radar_config.occupancy_delay_s = 30; // 30 seconds
    g_radar_config.motion_detected = false;
    g_radar_config.presence_confirmed = false;
    g_radar_config.occupancy_state = false;
    g_radar_config.enable_uart_plotting = false;

    last_confirmed_presence_timestamp = get_time_s();
}

radar_config_t* radar_config_get(void) {
    return &g_radar_config;
}

// --- SETTERS ---
void radar_config_set_uart_plotting(bool enable) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.enable_uart_plotting = enable;
    xSemaphoreGive(config_mutex);
    ESP_LOGI(TAG, "UART plotting %s", enable ? "enabled" : "disabled");
}

void radar_config_set_frame_delay(uint32_t delay_ms) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.frame_delay_ms = delay_ms;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_presence_params(float bg_alpha, uint32_t recal_interval_s) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.background_alpha = bg_alpha;
    g_radar_config.recalibration_interval_s = recal_interval_s;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_cfar_params(float near_bias, float far_bias, uint8_t guards, uint8_t refs) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.near_range_bias_db = near_bias;
    g_radar_config.far_range_bias_db = far_bias;
    g_radar_config.presence_cfar_guards = guards;
    g_radar_config.presence_cfar_refs = refs;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_temporal_filter_params(uint8_t hist_len, uint8_t min_dets, float max_range_diff) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.history_len = hist_len;
    g_radar_config.min_detections_in_history = min_dets;
    g_radar_config.max_range_diff_m = max_range_diff;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_processing_params(uint8_t int_factor, uint8_t avg_window) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    if (int_factor > 0 && (M_CHIRPS % int_factor == 0)) {
        g_radar_config.coherent_integration_factor = int_factor;
    }
    // Ensure moving average window is odd
    if (avg_window > 0) {
        g_radar_config.moving_avg_window_size = (avg_window % 2 == 0) ? avg_window + 1 : avg_window;
    }
    xSemaphoreGive(config_mutex);
    ESP_LOGI(TAG, "Processing params set: Integration Factor=%d, Avg Window=%d", g_radar_config.coherent_integration_factor, g_radar_config.moving_avg_window_size);
}

void radar_config_set_occupancy_delay(uint16_t delay_s) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.occupancy_delay_s = delay_s;
    xSemaphoreGive(config_mutex);
    ESP_LOGI(TAG, "Occupancy delay set to %d seconds", delay_s);
}


// --- STATE MANAGEMENT ---
void radar_config_set_motion_detected(bool detected) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.motion_detected = detected;
    xSemaphoreGive(config_mutex);
}

void radar_config_set_presence_confirmed(bool confirmed) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    g_radar_config.presence_confirmed = confirmed;
    xSemaphoreGive(config_mutex);
}

bool radar_config_get_motion_detected_safe(void) {
    bool detected;
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    detected = g_radar_config.motion_detected;
    xSemaphoreGive(config_mutex);
    return detected;
}

bool radar_config_get_presence_confirmed_safe(void) {
    bool confirmed;
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    confirmed = g_radar_config.presence_confirmed;
    xSemaphoreGive(config_mutex);
    return confirmed;
}

bool radar_config_get_occupancy_state_safe(void) {
    bool occupied;
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    occupied = g_radar_config.occupancy_state;
    xSemaphoreGive(config_mutex);
    return occupied;
}

// --- I2C MASTER INTERRUPT ---
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
    // Initial state is low (vacant)
    gpio_set_level((gpio_num_t)I2C_MASTER_IRQ_PIN, 0);
    interrupt_pin_initialized = true;
    last_interrupt_pin_state = false;
    ESP_LOGI(TAG, "I2C master interrupt pin GPIO %d initialized", I2C_MASTER_IRQ_PIN);
    return ESP_OK;
}
