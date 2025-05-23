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
#include "bgt60tr13c_regs.h" 
#include "freertos/semphr.h"

static const char * TAG = "spi-test-runner";

#define SPI_HOST SPI2_HOST
#define SPI_CLK_SPEED 20 

#define SPI_CS_PIN GPIO_NUM_13 
#define SPI_SCK_PIN GPIO_NUM_27 
#define SPI_MOSI_PIN GPIO_NUM_25 
#define SPI_MISO_PIN GPIO_NUM_26 
#define RADAR_IRQ_PIN GPIO_NUM_4 
#define RADAR_RESET_PIN GPIO_NUM_14

// How many samples to print before yielding
#define PRINT_CHUNK_BEFORE_YIELD 32 // Reduced from 128 to yield more often

SemaphoreHandle_t xSemaphore = NULL;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void xensiv_bgt60tr13c_radar_task(void *pvParameters) {
    ESP_LOGI(TAG, "starting radar task for continuous frame capture");

    uint32_t frame_size_samples = 0;
    esp_err_t err_check;

    err_check = get_frame_size(&frame_size_samples);
    if (err_check != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame_size_samples: %s", esp_err_to_name(err_check));
        vTaskDelete(NULL); return;
    }

    if (frame_size_samples == 0) {
        ESP_LOGE(TAG, "Frame size (samples) is 0. Check radar configuration.");
        vTaskDelete(NULL); return;
    }

    uint32_t temp_buf_len_bytes = 255; 

    uint16_t *frame_buf = (uint16_t *)malloc(frame_size_samples * sizeof(uint16_t));
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len_bytes * sizeof(uint8_t));

    if (frame_buf == NULL || temp_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        if (xSemaphore != NULL) {
            vSemaphoreDelete(xSemaphore);
            xSemaphore = NULL;
        }
        vTaskDelete(NULL);
        return;
    }
    memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));
    
    ESP_LOGI(TAG, "frame_size (number of 16-bit samples for frame_buf): %lu", frame_size_samples);
    ESP_LOGI(TAG, "frame_buf size (bytes): %lu", frame_size_samples * sizeof(uint16_t));
    ESP_LOGI(TAG, "temp_buf_len_bytes (bytes to read per FIFO transaction): %lu", temp_buf_len_bytes);

    err_check = xensiv_bgt60tr13c_start_frame_capture();
    if (err_check != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start frame capture: %s", esp_err_to_name(err_check));
        free(frame_buf);
        free(temp_buf);
        if (xSemaphore != NULL) { vSemaphoreDelete(xSemaphore); xSemaphore = NULL; }
        vTaskDelete(NULL); return;
    }

    uint32_t current_idx = 0;
    uint32_t total_frames_collected_count = 0;

    for (;;) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            uint32_t num_words_to_request_this_burst = temp_buf_len_bytes / 3;
            uint32_t len_val_for_driver_call = 0;

            if (num_words_to_request_this_burst > 0) {
                if (num_words_to_request_this_burst > 128) { 
                    num_words_to_request_this_burst = 128;
                }
                len_val_for_driver_call = num_words_to_request_this_burst - 1;
            }
            
            memset(temp_buf, 0, temp_buf_len_bytes); 
            err_check = xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, len_val_for_driver_call);

            if (err_check != ESP_OK) {
                ESP_LOGE(TAG, "FIFO read error: %s. Resetting frame progress. Total frames before error: %lu", esp_err_to_name(err_check), total_frames_collected_count);
                current_idx = 0;
                memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));
                // If FIFO errors persist, more robust recovery might be needed here,
                // such as resetting radar FIFO and re-triggering frame capture.
                // For now, just reset buffer and hope next IRQ comes.
                vTaskDelay(pdMS_TO_TICKS(100)); // Delay after error before trying again
                ESP_LOGI(TAG, "Attempting to re-start frame capture after FIFO error.");
                xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO); // Reset FIFO
                vTaskDelay(pdMS_TO_TICKS(10));
                xensiv_bgt60tr13c_start_frame_capture(); // Re-trigger
                continue;
            }

            for (uint32_t i = 0; (i + 2) < temp_buf_len_bytes; i += 3) {
                if (current_idx < frame_size_samples) {
                    frame_buf[current_idx] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                    current_idx++;
                } else {
                    break; 
                }

                if (current_idx < frame_size_samples) {
                    frame_buf[current_idx] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                    current_idx++;
                } else {
                    break;
                }
            }

            if (current_idx >= frame_size_samples) {
                total_frames_collected_count++;
                ESP_LOGI(TAG, "Full frame %lu collected (%lu samples). Printing to UART...", total_frames_collected_count, frame_size_samples);

                printf("Frame %lu: [", total_frames_collected_count);
                for (uint32_t i = 0; i < frame_size_samples; i++) {
                    printf("%d", frame_buf[i]);
                    if (i < frame_size_samples - 1) {
                        printf(", ");
                    }
                    // Yield more frequently during printing
                    if ((i > 0 && (i % PRINT_CHUNK_BEFORE_YIELD == 0))) {
                        vTaskDelay(pdMS_TO_TICKS(1)); // Yield for 1ms
                    }
                }
                printf("]\n");
                fflush(stdout); // Ensure data is sent out
                ESP_LOGI(TAG, "Finished printing frame %lu.", total_frames_collected_count);

                // Significant delay after printing a full frame to allow system to recover
                // and hopefully allow the radar to be ready for the next sequence.
                ESP_LOGI(TAG, "Delaying for system recovery after print...");
                vTaskDelay(pdMS_TO_TICKS(500)); // Increase this delay if it still stops. Try 500ms, 1s, or more.

                current_idx = 0;
                memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t)); 
                
                // After a long print, radar FIFO is likely overflowed and needs reset.
                ESP_LOGI(TAG, "Resetting FIFO and re-triggering frame capture for next frame.");
                err_check = xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
                if (err_check != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to soft reset FIFO post-print: %s", esp_err_to_name(err_check));
                }
                vTaskDelay(pdMS_TO_TICKS(20)); // Allow reset to complete

                err_check = xensiv_bgt60tr13c_start_frame_capture();
                if (err_check != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to re-start frame capture post-print: %s", esp_err_to_name(err_check));
                }
                ESP_LOGI(TAG, "Ready for next frame acquisition.");
            }
        }
    }

    free(frame_buf);
    free(temp_buf);
    if (xSemaphore != NULL) {
        vSemaphoreDelete(xSemaphore);
        xSemaphore = NULL;
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    // NOTE: Ensure UART baud rate is set high in menuconfig for faster printing.
    // e.g., idf.py menuconfig -> Component config -> ESP System Settings -> Channel for console output -> UART Baud Rate
    // Set to 921600 or higher if your setup supports it.

    esp_err_t ret; 
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 
    };

    ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
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
    ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
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
        ESP_LOGI(TAG, "Radar configured with settings from bgt60tr13c_config.h");
    }
    
    ESP_LOGI(TAG, "Setting CCR2 for endless frames.");
    uint32_t ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "Original CCR2 value: 0x%08lX", ccr2_val);

    uint32_t ccr2_frame_len_part = (ccr2_val & 0x00FFF000); 
    uint32_t ccr2_to_write = ccr2_frame_len_part; 
    
    ESP_LOGI(TAG, "CCR2 value to write for endless frames (MAX_FRAME_CNT=0): 0x%08lX", ccr2_to_write);
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_CCR2, ccr2_to_write, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CCR2 for endless frames: %s.", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Successfully set CCR2 for endless frames.");
    }
    uint32_t new_ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "New CCR2 value read back: 0x%08lX", new_ccr2_val);

    if ((new_ccr2_val & 0x00000FFF) != 0) {
        ESP_LOGW(TAG, "MAX_FRAME_CNT in CCR2 (0x%08lX) is not 0 after setting! Value: 0x%lX. Continuous mode might not be enabled.", new_ccr2_val, (new_ccr2_val & 0xFFF));
    } else {
        ESP_LOGI(TAG, "MAX_FRAME_CNT in CCR2 is 0. Configured for endless frames.");
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
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    ret = gpio_install_isr_service(0); 
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { 
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return;
    }
    ret = gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        if (ret != ESP_ERR_INVALID_STATE) gpio_uninstall_isr_service(); 
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return;
    }

    ESP_LOGI(TAG, "Creating radar task");
    BaseType_t task_created = xTaskCreate(xensiv_bgt60tr13c_radar_task, "radar-task", 8192 * 2, NULL, 5, NULL); // Priority 5
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create radar task");
        gpio_isr_handler_remove(RADAR_IRQ_PIN);
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
    } else {
        ESP_LOGI(TAG, "Radar task created");
    }
    ESP_LOGI(TAG, "app_main finished setup.");
}