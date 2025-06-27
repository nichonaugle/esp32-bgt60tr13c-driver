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
static SemaphoreHandle_t xFrameTriggerSemaphore = NULL; // Given by main to start a frame
static SemaphoreHandle_t xRadarReadySemaphore = NULL;   // Given by this task when ready for next trigger

// --- Global Variables ---
static radar_buffers_t g_radar_buffers = {0};
static uint32_t g_frame_size_samples = 0;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xFifoIRQSemaphore, NULL);
}

void radar_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radar acquisition task started for on-demand frame capture.");

    // Get frame size from the driver
    ESP_ERROR_CHECK(get_frame_size(&g_frame_size_samples));
    ESP_LOGI(TAG, "Frame size determined to be: %lu samples", g_frame_size_samples);

    // Initialize processing buffers (from radar_processing.c)
    if (radar_processing_init(&g_radar_buffers, g_frame_size_samples) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing buffers. Deleting task.");
        vTaskDelete(NULL);
        return;
    }

    // Allocate a temporary buffer for FIFO reads
    uint32_t temp_buf_len_bytes = 4092;
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len_bytes);
    if (!temp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp_buf. Deleting task.");
        radar_processing_cleanup(&g_radar_buffers);
        vTaskDelete(NULL);
        return;
    }
    
    uint32_t total_frames_collected_count = 0;

    for (;;) {
        // 1. Signal to main() that we are ready for the next trigger.
        // This is done *before* waiting to prevent a deadlock.
        xSemaphoreGive(xRadarReadySemaphore);

        // 2. Wait indefinitely for the frame trigger from main().
        ESP_LOGD(TAG, "Waiting for frame trigger from main task...");
        if (xSemaphoreTake(xFrameTriggerSemaphore, portMAX_DELAY) == pdTRUE) {
            total_frames_collected_count++;
            ESP_LOGI(TAG, "Frame trigger received. Starting frame capture #%lu.", total_frames_collected_count);

            // Clear any stale IRQ semaphores from a previous run before starting
            while(xSemaphoreTake(xFifoIRQSemaphore, (TickType_t)0) == pdTRUE);

            // Start the frame capture on the radar chip
            ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture());

            uint32_t current_idx = 0;
            bool frame_collection_error = false;
            uint32_t fifo_timeouts = 0;
            const uint32_t MAX_FIFO_TIMEOUTS = 15; // Increased tolerance for timeouts

            // 3. Enter the data collection loop
            while (current_idx < g_frame_size_samples) {
                if (xSemaphoreTake(xFifoIRQSemaphore, pdMS_TO_TICKS(800)) == pdTRUE) {
                    fifo_timeouts = 0; // Reset timeout counter on successful IRQ
                    
                    memset(temp_buf, 0, temp_buf_len_bytes);
                    esp_err_t read_err = xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, 0);

                    if (read_err != ESP_OK) {
                        ESP_LOGE(TAG, "FIFO read error: %s. Frame %lu aborted.", esp_err_to_name(read_err), total_frames_collected_count);
                        frame_collection_error = true;
                        break;
                    }

                    // Unpack the 12-bit data which is packed into a 3-byte format
                    uint32_t bytes_to_process = temp_buf_len_bytes - (temp_buf_len_bytes % 3);
                    for (uint32_t i = 0; i < bytes_to_process && current_idx < g_frame_size_samples; i += 3) {
                        if (current_idx < g_frame_size_samples) {
                            g_radar_buffers.frame_buf_uint16[current_idx++] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                        }
                        if (current_idx < g_frame_size_samples) {
                            g_radar_buffers.frame_buf_uint16[current_idx++] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                        }
                    }
                    ESP_LOGD(TAG, "IRQ data processed. Total collected: %lu/%lu", current_idx, g_frame_size_samples);

                } else {
                    fifo_timeouts++;
                    ESP_LOGW(TAG, "Timeout waiting for FIFO IRQ (%lu/%lu samples collected). Attempt %lu/%lu.",
                             current_idx, g_frame_size_samples, fifo_timeouts, MAX_FIFO_TIMEOUTS);
                    if (fifo_timeouts >= MAX_FIFO_TIMEOUTS) {
                        ESP_LOGE(TAG, "Aborting frame %lu due to too many FIFO IRQ timeouts.", total_frames_collected_count);
                        frame_collection_error = true;
                        break;
                    }
                }
            }

            // 4. Process the collected frame if it was successful
            if (!frame_collection_error && current_idx >= g_frame_size_samples) {
                ESP_LOGI(TAG, "Full frame %lu collected (%lu samples). Processing...", total_frames_collected_count, g_frame_size_samples);
                process_radar_frame(&g_radar_buffers, total_frames_collected_count);
            } else {
                ESP_LOGE(TAG, "Frame %lu was not fully collected (got %lu/%lu samples). Not processing.",
                         total_frames_collected_count, current_idx, g_frame_size_samples);
            }

            // 5. Reset FIFO to ensure chip enters deep sleep and is ready for the next trigger
            ESP_LOGD(TAG, "Resetting FIFO post-frame.");
            ESP_ERROR_CHECK(xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO));

            ESP_LOGI(TAG, "Frame acquisition cycle complete. Waiting for next trigger.");
        }
    }

    // Cleanup should never be reached in this design, but is good practice
    free(temp_buf);
    radar_processing_cleanup(&g_radar_buffers);
    vSemaphoreDelete(xFifoIRQSemaphore);
    vSemaphoreDelete(xFrameTriggerSemaphore);
    vSemaphoreDelete(xRadarReadySemaphore);
    vTaskDelete(NULL);
}

void radar_acquisition_task_create(void) {
    // Create all necessary semaphores for synchronization
    xFifoIRQSemaphore = xSemaphoreCreateBinary();
    xFrameTriggerSemaphore = xSemaphoreCreateBinary();
    xRadarReadySemaphore = xSemaphoreCreateBinary();

    if (!xFifoIRQSemaphore || !xFrameTriggerSemaphore || !xRadarReadySemaphore) {
        ESP_LOGE(TAG, "Failed to create acquisition semaphores!");
        // Depending on desired robustness, you might want to halt here
        return;
    }
    
    // Create the radar acquisition task
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