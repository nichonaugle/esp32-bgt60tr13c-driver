#include "radar_acquisition.h"
#include "radar_config.h"
#include "bgt60tr13c_driver.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "RADAR_ACQUISITION";

// --- RTOS Handles for Synchronization ---
static SemaphoreHandle_t xFifoIRQSemaphore = NULL;
static SemaphoreHandle_t xFrameTriggerSemaphore = NULL;
static SemaphoreHandle_t xRadarReadySemaphore = NULL;

// --- Global Variables ---
static radar_buffers_t g_radar_buffers = {0};
static uint32_t g_frame_size_samples = 0;
static radar_config_t *g_config = NULL;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xFifoIRQSemaphore, NULL);
}

void radar_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radar acquisition task started.");

    g_config = radar_config_get();

    ESP_ERROR_CHECK(get_frame_size(&g_frame_size_samples));
    ESP_LOGI(TAG, "Frame size determined to be: %lu samples", g_frame_size_samples);

    if (radar_processing_init(&g_radar_buffers, g_config, g_frame_size_samples) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing buffers. Deleting task.");
        vTaskDelete(NULL);
        return;
    }

    uint32_t temp_buf_len_bytes = (M_CHIRPS * N_SAMPLES_PER_CHIRP * 3) / 2;
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len_bytes);
    if (!temp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp_buf. Deleting task.");
        radar_processing_cleanup(&g_radar_buffers, g_config);
        vTaskDelete(NULL);
        return;
    }
    
    uint32_t total_frames_collected_count = 0;

    for (;;) {
        xSemaphoreGive(xRadarReadySemaphore);

        ESP_LOGD(TAG, "Waiting for frame trigger from main task...");
        if (xSemaphoreTake(xFrameTriggerSemaphore, portMAX_DELAY) == pdTRUE) {
            total_frames_collected_count++;
            ESP_LOGI(TAG, "Frame trigger received. Starting frame capture #%lu.", total_frames_collected_count);

            while(xSemaphoreTake(xFifoIRQSemaphore, (TickType_t)0) == pdTRUE);

            ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture());

            // Allow radar time for PLL stabilization and chirp sequence start up.
            vTaskDelay(pdMS_TO_TICKS(100));

            uint32_t current_idx = 0;
            bool frame_collection_error = false;
            uint32_t fifo_timeouts = 0;
            const uint32_t MAX_FIFO_TIMEOUTS = 15;

            while (current_idx < g_frame_size_samples) {
                if (xSemaphoreTake(xFifoIRQSemaphore, pdMS_TO_TICKS(800)) == pdTRUE) {
                    fifo_timeouts = 0;
                    
                    memset(temp_buf, 0, temp_buf_len_bytes);
                    esp_err_t read_err = xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, 0);

                    if (read_err != ESP_OK) {
                        ESP_LOGE(TAG, "FIFO read error: %s. Frame %lu aborted.", esp_err_to_name(read_err), total_frames_collected_count);
                        frame_collection_error = true;
                        break;
                    }

                    uint32_t bytes_to_process = temp_buf_len_bytes - (temp_buf_len_bytes % 3);
                    for (uint32_t i = 0; i < bytes_to_process; i += 3) {
                        if (current_idx < g_frame_size_samples) {
                            g_radar_buffers.raw_frame_buf[current_idx++] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                        }
                        if (current_idx < g_frame_size_samples) {
                            g_radar_buffers.raw_frame_buf[current_idx++] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                        }
                    }
                    ESP_LOGD(TAG, "IRQ data processed. Total collected: %lu/%lu", current_idx, g_frame_size_samples);
                } else {
                    fifo_timeouts++;
                    ESP_LOGW(TAG, "Timeout waiting for FIFO IRQ (%lu/%lu samples). Attempt %lu/%lu.",
                             current_idx, g_frame_size_samples, fifo_timeouts, MAX_FIFO_TIMEOUTS);
                    if (fifo_timeouts >= MAX_FIFO_TIMEOUTS) {
                        ESP_LOGE(TAG, "Aborting frame %lu due to too many FIFO timeouts.", total_frames_collected_count);
                        frame_collection_error = true;
                        break;
                    }
                }
            }

            if (!frame_collection_error && current_idx >= g_frame_size_samples) {
                ESP_LOGI(TAG, "Full frame %lu collected (%lu samples). Processing...", total_frames_collected_count, g_frame_size_samples);
                process_radar_frame(&g_radar_buffers, g_config, total_frames_collected_count);
            } else {
                ESP_LOGE(TAG, "Frame %lu was not fully collected (got %lu/%lu samples). Not processing.",
                         total_frames_collected_count, current_idx, g_frame_size_samples);
            }

            ESP_LOGD(TAG, "Resetting FIFO post-frame.");
            ESP_ERROR_CHECK(xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO));
            
            // Allow hardware to settle after FIFO reset.
            vTaskDelay(pdMS_TO_TICKS(20));

            ESP_LOGI(TAG, "Frame acquisition cycle complete.");
        }
    }

    free(temp_buf);
    radar_processing_cleanup(&g_radar_buffers, g_config);
    vSemaphoreDelete(xFifoIRQSemaphore);
    vSemaphoreDelete(xFrameTriggerSemaphore);
    vSemaphoreDelete(xRadarReadySemaphore);
    vTaskDelete(NULL);
}

void radar_acquisition_task_create(void) {
    xFifoIRQSemaphore = xSemaphoreCreateBinary();
    xFrameTriggerSemaphore = xSemaphoreCreateBinary();
    xRadarReadySemaphore = xSemaphoreCreateBinary();

    if (!xFifoIRQSemaphore || !xFrameTriggerSemaphore || !xRadarReadySemaphore) {
        ESP_LOGE(TAG, "Failed to create acquisition semaphores!");
        return;
    }
    
    xTaskCreate(radar_acquisition_task, "radar_acquisition_task", 12288, NULL, 5, NULL);
}

void radar_acquisition_trigger_frame(void) {
    if (xFrameTriggerSemaphore != NULL) {
        xSemaphoreGive(xFrameTriggerSemaphore);
    }
}

BaseType_t radar_acquisition_wait_for_ready(TickType_t xTicksToWait) {
    if (xRadarReadySemaphore != NULL) {
        return xSemaphoreTake(xRadarReadySemaphore, xTicksToWait);
    }
    return pdFALSE;
}