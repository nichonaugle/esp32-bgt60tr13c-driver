#include "radar_acquisition.h"
#include "radar_config.h"
#include "bgt60tr13c_driver.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "RADAR_ACQUISITION";

// --- RTOS Handles ---
SemaphoreHandle_t xFifoIRQSemaphore = NULL;

// --- Global Variables ---
static radar_buffers_t g_radar_buffers = {0};
static uint32_t g_frame_size_samples = 0;
static uint32_t g_irq_frame_size = 0;
static uint32_t g_frame_count = 0;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xFifoIRQSemaphore, NULL);
}

void radar_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radar acquisition task started.");

    // Get both frame size and IRQ frame size
    ESP_ERROR_CHECK(get_frame_size(&g_frame_size_samples));
    ESP_ERROR_CHECK(get_interrupt_frame_size_trigger(&g_irq_frame_size));
    
    ESP_LOGI(TAG, "Frame size: %lu samples", g_frame_size_samples);
    ESP_LOGI(TAG, "IRQ frame size: %lu", g_irq_frame_size);

    // Initialize processing buffers
    if (radar_processing_init(&g_radar_buffers, g_frame_size_samples) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing buffers. Exiting.");
        vTaskDelete(NULL);
        return;
    }

    // Use larger buffer size for fewer IRQs
    uint32_t temp_buf_len_bytes = 4092;
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len_bytes);
    if (!temp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp_buf. Exiting.");
        radar_processing_cleanup(&g_radar_buffers);
        vTaskDelete(NULL); 
        return;
    }
    
    for (;;) { 
        radar_config_t *config = radar_config_get();
        g_frame_count++;
        ESP_LOGI(TAG, "Starting frame capture #%lu.", g_frame_count);
            
        // Clear semaphore before starting frame capture
        while(xSemaphoreTake(xFifoIRQSemaphore, 0) == pdTRUE);
        
        ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture()); 
        
        // Small delay to avoid FIFO issues
        vTaskDelay(pdMS_TO_TICKS(10));

        uint32_t current_idx = 0;
        bool frame_error = false;
        uint32_t timeouts = 0;
        uint32_t irq_count = 0;

        while (current_idx < g_frame_size_samples) {
            if (xSemaphoreTake(xFifoIRQSemaphore, pdMS_TO_TICKS(800)) == pdTRUE) {
                timeouts = 0; 
                irq_count++;
                memset(temp_buf, 0, temp_buf_len_bytes);
                
                // Use g_irq_frame_size parameter
                if (xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, g_irq_frame_size) != ESP_OK) {
                    ESP_LOGE(TAG, "FIFO read error for frame %lu.", g_frame_count);
                    frame_error = true; 
                    break; 
                }
                
                uint32_t samples_this_irq = 0;
                // Improved unpacking with better bounds checking
                uint32_t bytes_to_process = temp_buf_len_bytes - (temp_buf_len_bytes % 3); // Ensure multiple of 3
                for (uint32_t i = 0; i < bytes_to_process && current_idx < g_frame_size_samples; i += 3) {
                    if (current_idx < g_frame_size_samples) {
                        g_radar_buffers.frame_buf_uint16[current_idx] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                        current_idx++;
                        samples_this_irq++;
                    }
                    if (current_idx < g_frame_size_samples) {
                        g_radar_buffers.frame_buf_uint16[current_idx] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                        current_idx++;
                        samples_this_irq++;
                    }
                }
                ESP_LOGI(TAG, "IRQ #%lu: Read %lu samples. Total collected: %lu/%lu", 
                         irq_count, samples_this_irq, current_idx, g_frame_size_samples);
            } else { 
                timeouts++;
                ESP_LOGW(TAG, "Timeout waiting for FIFO IRQ (%lu/%lu).", current_idx, g_frame_size_samples); 
                if (timeouts >= 12) {
                    ESP_LOGE(TAG, "Frame %lu incomplete.", g_frame_count); 
                    frame_error = true; 
                    break; 
                }
            }
        } 

        if (!frame_error) {
            ESP_LOGI(TAG, "Full frame %lu collected. Processing...", g_frame_count);
            process_radar_frame(&g_radar_buffers, g_frame_count);
        } else {
            ESP_LOGE(TAG, "Frame %lu not fully collected.", g_frame_count);
        }

        // Reset FIFO and add delay between frames (use dynamic config)
        ESP_ERROR_CHECK(xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO));
        vTaskDelay(pdMS_TO_TICKS(config->frame_delay_ms));
    } 
    
    free(temp_buf);
    radar_processing_cleanup(&g_radar_buffers);
    vTaskDelete(NULL);
}

void radar_acquisition_task_create(void) {
    // Create semaphores
    xFifoIRQSemaphore = xSemaphoreCreateBinary();
    
    // Create the radar acquisition task
    xTaskCreate(radar_acquisition_task, "radar_acquisition_task", 12288, NULL, 5, NULL);
}