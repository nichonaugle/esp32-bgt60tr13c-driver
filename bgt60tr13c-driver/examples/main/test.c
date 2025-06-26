#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "bgt60tr13c_driver.h" 
#include "freertos/semphr.h"

static const char * TAG = "spi-test-runner";

#define SPI_CLK_SPEED 20

#define SPI_CS_PIN GPIO_NUM_13
#define SPI_SCK_PIN GPIO_NUM_27
#define SPI_MOSI_PIN GPIO_NUM_25
#define SPI_MISO_PIN GPIO_NUM_26
#define RADAR_IRQ_PIN GPIO_NUM_4
#define RADAR_RESET_PIN GPIO_NUM_14 
#define PRINT_CHUNK_BEFORE_YIELD 32

SemaphoreHandle_t xSemaphore = NULL; 
SemaphoreHandle_t xFrameTriggerSemaphore = NULL; 

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void xensiv_bgt60tr13c_radar_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radar task started for on-demand frame capture.");

    uint32_t frame_size_samples = 0;
    esp_err_t err_check;

    err_check = get_frame_size(&frame_size_samples);
    if (err_check != ESP_OK || frame_size_samples == 0) {
        ESP_LOGE(TAG, "Failed to get valid frame_size_samples: %s, size: %lu. Exiting task.",
                 esp_err_to_name(err_check), frame_size_samples);
        vTaskDelete(NULL); return;
    }

    uint32_t temp_buf_len_bytes = 1026;

    uint16_t *frame_buf = (uint16_t *)malloc(frame_size_samples * sizeof(uint16_t));
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len_bytes * sizeof(uint8_t));

    if (frame_buf == NULL || temp_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers. Exiting task.");
        free(frame_buf);
        free(temp_buf);
        vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "Frame buffer allocated for %lu samples (%lu bytes). temp_buf %lu bytes.",
             frame_size_samples, frame_size_samples * sizeof(uint16_t), temp_buf_len_bytes);

    uint32_t total_frames_collected_count = 0;

    for (;;) { 
        ESP_LOGI(TAG, "Waiting for frame trigger...");
        if (xSemaphoreTake(xFrameTriggerSemaphore, portMAX_DELAY) == pdTRUE) {
            total_frames_collected_count++;
            ESP_LOGI(TAG, "Frame trigger received for frame attempt #%lu.", total_frames_collected_count);

            // Clear any pending IRQ semaphores (but DON'T reset FIFO yet)
            while(xSemaphoreTake(xSemaphore, (TickType_t)0) == pdTRUE); 

            // Start frame capture WITHOUT pre-reset
            ESP_LOGI(TAG, "Starting frame capture command.");
            err_check = xensiv_bgt60tr13c_start_frame_capture(); 
            if (err_check != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start frame capture: %s. Skipping.", esp_err_to_name(err_check));
                continue; 
            }

            // CRITICAL: Allow radar initialization time before expecting data
            // This covers T_INIT0 + T_INIT1 + T_START timing from datasheet
            //ESP_LOGI(TAG, "Allowing radar initialization time (T_INIT0 + T_INIT1 + T_START)...");
            vTaskDelay(pdMS_TO_TICKS(50));

            uint32_t current_idx = 0;
            memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));
            bool frame_collection_error = false;
            uint32_t fifo_timeouts = 0;
            const uint32_t MAX_FIFO_TIMEOUTS = 15; // Increased for lower FIFO_CREF
            uint32_t irq_count = 0; // For debugging IRQ behavior

            // Modified collection loop - adjusted timeout for 32-sample threshold
            while (current_idx < frame_size_samples) {
                if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(800)) == pdTRUE) { 
                    fifo_timeouts = 0; 
                    irq_count++;
                    
                    memset(temp_buf, 0, temp_buf_len_bytes);
                    err_check = xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, 0);

                    if (err_check != ESP_OK) {
                        ESP_LOGE(TAG, "FIFO read error: %s. Frame %lu incomplete.",
                                 esp_err_to_name(err_check), total_frames_collected_count);
                        frame_collection_error = true;
                        break; 
                    }

                    // Count how much valid data we actually got from FIFO
                    uint32_t samples_read_this_irq = 0;
                    uint32_t current_idx_before = current_idx;

                    // Process the 12-bit data packed in 3-byte format
                    for (uint32_t i = 0; (i + 2) < temp_buf_len_bytes; i += 3) {
                        if (current_idx < frame_size_samples) {
                            frame_buf[current_idx] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                            current_idx++;
                            samples_read_this_irq++;
                        } else { break; }

                        if (current_idx < frame_size_samples) {
                            frame_buf[current_idx] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                            current_idx++;
                            samples_read_this_irq++;
                        } else { break; }
                    }
                    
                    // Log how much data we got per IRQ
                    ESP_LOGI(TAG, "IRQ #%lu: Read %lu samples (%lu bytes). Total: %lu/%lu samples", 
                             irq_count, samples_read_this_irq, (samples_read_this_irq * 3 / 2), 
                             current_idx, frame_size_samples);
                    
                    if (current_idx >= frame_size_samples) {
                         ESP_LOGD(TAG, "All samples for frame collected based on current_idx.");
                         break; 
                    }

                } else { 
                    fifo_timeouts++;
                    ESP_LOGW(TAG, "Timeout waiting for FIFO IRQ (%lu/%lu samples collected). IRQ count: %lu, Attempt %lu/%lu for frame %lu.",
                             current_idx, frame_size_samples, irq_count, fifo_timeouts, MAX_FIFO_TIMEOUTS, total_frames_collected_count); 
                    if (fifo_timeouts >= MAX_FIFO_TIMEOUTS) {
                        ESP_LOGE(TAG, "Too many FIFO IRQ timeouts. Frame %lu declared incomplete after %lu IRQs.", 
                                 total_frames_collected_count, irq_count);
                        frame_collection_error = true;
                        break; 
                    }
                }
            } 

            if (!frame_collection_error && current_idx >= frame_size_samples) {
                ESP_LOGI(TAG, "Full frame %lu collected (%lu samples) with %lu IRQs. Average: %lu samples/IRQ",
                         total_frames_collected_count, frame_size_samples, irq_count, 
                         (irq_count > 0) ? (frame_size_samples / irq_count) : 0);

                printf("Frame %lu: [", total_frames_collected_count);
                for (uint32_t i = 0; i < frame_size_samples; i++) {
                    printf("%d", frame_buf[i]);
                    if (i < frame_size_samples - 1) {
                        printf(", ");
                    }
                    if ((i > 0 && (i % PRINT_CHUNK_BEFORE_YIELD == 0))) {
                        vTaskDelay(pdMS_TO_TICKS(1)); 
                    }
                }
                printf("]\n");
                fflush(stdout);
                ESP_LOGI(TAG, "Finished printing frame %lu.", total_frames_collected_count);
                ESP_LOGI(TAG, "System delay after print...");
                vTaskDelay(pdMS_TO_TICKS(50));

            } else {
                ESP_LOGE(TAG, "Frame %lu was not fully collected or had errors (collected %lu/%lu samples) after %lu IRQs. Not printing.",
                         total_frames_collected_count, current_idx, frame_size_samples, irq_count);
                current_idx = 0; 
                memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));
            }

            // IMPORTANT: Only reset FIFO AFTER frame completion (success or failure)
            // This is when the chip enters Deep Sleep mode per datasheet (MAX_FRAME_CNT=1 reached)
            ESP_LOGI(TAG, "Resetting FIFO after frame completion (chip entering Deep Sleep per MAX_FRAME_CNT=1).");
            err_check = xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
            if (err_check != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reset FIFO post-frame: %s", esp_err_to_name(err_check));
            }
            //vTaskDelay(pdMS_TO_TICKS(5));

            ESP_LOGI(TAG, "Frame acquisition cycle complete. Radar should be in deep sleep (MAX_FRAME_CNT=1 reached).");
        } 
    } 

    free(frame_buf);
    free(temp_buf);
    ESP_LOGI(TAG, "Radar task exiting."); 
    vTaskDelete(NULL);
}

void app_main(void) {
    uart_set_baudrate(UART_NUM_0, 921600);
    esp_err_t ret;
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Successfully initialized SPI bus!");

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 7,
    };
    ret = xensiv_bgt60tr13c_init(SPI2_HOST, &dev_config); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "xensiv_bgt60tr13c_init failed: %s.", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "xensiv_bgt60tr13c_init successful.");
    }

    ESP_LOGI(TAG, "Configuring radar with settings from bgt60tr13c_config.h");
    ret = xensiv_bgt60tr13c_configure(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "xensiv_bgt60tr13c_configure failed: %s", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "Radar configured with base settings from bgt60tr13c_config.h");
    }

    // --- Modify SFCTL:FIFO_CREF for earlier IRQ generation ---
    ESP_LOGI(TAG, "Modifying SFCTL:FIFO_CREF for earlier IRQ generation.");
    uint32_t sfctl_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_SFCTL);
    ESP_LOGI(TAG, "Original SFCTL value: 0x%08lX", sfctl_val);

    // Use 32 samples instead of 64 for faster IRQ response to catch early chirps
    uint32_t new_sfctl_data = (sfctl_val & ~0x1FFFU) | 0x020U;  // 0x20 = 32 samples

    ESP_LOGI(TAG, "SFCTL value to write with FIFO_CREF=0x20 (32 samples): 0x%08lX", new_sfctl_data);
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, new_sfctl_data, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SFCTL:FIFO_CREF: %s.", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SFCTL:FIFO_CREF set to 0x%lX (32 decimal).", (xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_SFCTL) & 0x1FFFU) );
    }

    ESP_LOGI(TAG, "Setting CCR2 for single frame capture (MAX_FRAME_CNT=1).");
    uint32_t ccr2_val_read = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "Original CCR2 value: 0x%08lX", ccr2_val_read);

    uint32_t ccr2_to_write = (ccr2_val_read & 0xFFFFF000) | 0x00000001;

    ESP_LOGI(TAG, "CCR2 value to write for single frame capture: 0x%08lX", ccr2_to_write);
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_CCR2, ccr2_to_write, true); 

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CCR2 for single frame: %s.", esp_err_to_name(ret));
        return; 
    } else {
        ESP_LOGI(TAG, "Successfully set CCR2 for single frame.");
        uint32_t new_ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
        ESP_LOGI(TAG, "New CCR2 value read back: 0x%08lX", new_ccr2_val);
        if ((new_ccr2_val & 0x00000FFF) != 1) {
            ESP_LOGW(TAG, "MAX_FRAME_CNT in CCR2 (0x%08lX) is not 1 after setting! Value: 0x%lX.", new_ccr2_val, (new_ccr2_val & 0xFFF));
        } else {
            ESP_LOGI(TAG, "MAX_FRAME_CNT in CCR2 is 1. Configured for single frame capture per trigger.");
        }
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary(); 
    if (xSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create xSemaphore (for FIFO IRQ)");
        return;
    }

    xFrameTriggerSemaphore = xSemaphoreCreateBinary(); 
    if (xFrameTriggerSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create xFrameTriggerSemaphore");
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        return;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        vSemaphoreDelete(xFrameTriggerSemaphore); xFrameTriggerSemaphore = NULL;
        return;
    }
    ret = gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        if (ret != ESP_ERR_INVALID_STATE && ret != ESP_ERR_NOT_FOUND) gpio_uninstall_isr_service(); 
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        vSemaphoreDelete(xFrameTriggerSemaphore); xFrameTriggerSemaphore = NULL;
        return;
    }

    ESP_LOGI(TAG, "Creating radar task");
    BaseType_t task_created = xTaskCreate(xensiv_bgt60tr13c_radar_task, "radar-task", 8192 * 2, NULL, 5, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create radar task");
        gpio_isr_handler_remove(RADAR_IRQ_PIN);
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        vSemaphoreDelete(xFrameTriggerSemaphore); xFrameTriggerSemaphore = NULL;
    } else {
        ESP_LOGI(TAG, "Radar task created");
    }
    
    ESP_LOGI(TAG, "app_main: Initial delay before starting frame triggers...");
    vTaskDelay(pdMS_TO_TICKS(2000)); 

    ESP_LOGI(TAG, "app_main: Starting continuous frame triggering.");
    for (;;) { // Infinite loop for continuous triggering
        static int frame_trigger_count_main = 0;
        frame_trigger_count_main++;
        ESP_LOGI(TAG, "app_main: Triggering frame capture (main loop iter #%d)", frame_trigger_count_main);
        xSemaphoreGive(xFrameTriggerSemaphore);
        vTaskDelay(pdMS_TO_TICKS(75)); // Wait between triggers
                                         // Adjust based on expected processing time and desired interval.
    }
}