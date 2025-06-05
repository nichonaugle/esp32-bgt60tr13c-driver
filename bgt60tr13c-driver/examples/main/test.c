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

// Define a safe chunk size for reads that is less than the FIFO capacity
#define FIFO_READ_CHUNK_SAMPLES 8192 // Read 8192 samples (4096 words) at a time
#define FIFO_READ_CHUNK_BYTES (FIFO_READ_CHUNK_SAMPLES * 3 / 2)

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t xFrameTriggerSemaphore = NULL;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void xensiv_bgt60tr13c_radar_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radar task started.");

    const uint32_t target_frame_size_samples = 128 * 64 * 3;
    const uint32_t raw_buffer_size = target_frame_size_samples * 3 / 2;

    uint8_t *raw_frame_buf = (uint8_t *)malloc(raw_buffer_size);
    if (raw_frame_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for raw_frame_buf. Exiting task.");
        vTaskDelete(NULL); return;
    }

    uint32_t total_frames_collected_count = 0;

    for (;;) {
        ESP_LOGI(TAG, "Waiting for frame trigger...");
        if (xSemaphoreTake(xFrameTriggerSemaphore, portMAX_DELAY) == pdTRUE) {
            total_frames_collected_count++;
            ESP_LOGI(TAG, "Frame trigger received for frame #%lu.", total_frames_collected_count);

            ESP_LOGI(TAG, "Starting frame capture.");
            esp_err_t err_check = xensiv_bgt60tr13c_start_frame_capture();
            if (err_check != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start frame capture: %s. Skipping.", esp_err_to_name(err_check));
                continue;
            }

            uint32_t bytes_collected = 0;
            
            while (bytes_collected < raw_buffer_size) {
                ESP_LOGI(TAG, "Waiting for IRQ for next chunk...");
                if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    uint32_t bytes_to_read = FIFO_READ_CHUNK_BYTES;
                    if (bytes_collected + bytes_to_read > raw_buffer_size) {
                        bytes_to_read = raw_buffer_size - bytes_collected;
                    }

                    err_check = xensiv_bgt60tr13c_fifo_read(&raw_frame_buf[bytes_collected], bytes_to_read, 0);
                    if (err_check == ESP_OK) {
                        bytes_collected += bytes_to_read;
                        ESP_LOGI(TAG, "Read chunk successfully. Total bytes collected: %lu/%lu", bytes_collected, raw_buffer_size);
                    } else {
                        ESP_LOGE(TAG, "FIFO read error. Halting collection.");
                        break; 
                    }
                } else {
                     ESP_LOGE(TAG, "Timeout waiting for FIFO IRQ. Frame may be incomplete.");
                     break; 
                }
            }
        
            ESP_LOGI(TAG, "Collection loop finished. Sending FIFO_RESET.");
            xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);

            if (bytes_collected >= raw_buffer_size) {
                ESP_LOGI(TAG, "Successfully collected full frame %lu (%lu bytes).", total_frames_collected_count, bytes_collected);
                
                // Print the unpacked 12-bit radar samples for the Python script
                printf("Frame %lu: [", total_frames_collected_count);
                
                // The buffer contains 12-bit samples packed into 3-byte chunks.
                // We iterate through the buffer, unpacking two samples at a time.
                for (uint32_t i = 0; i < raw_buffer_size; i += 3) {
                    // Unpack two 12-bit samples from three 8-bit bytes
                    uint16_t sample1 = (uint16_t)(raw_frame_buf[i] << 4) | (uint16_t)(raw_frame_buf[i+1] >> 4);
                    uint16_t sample2 = (uint16_t)((raw_frame_buf[i+1] & 0x0F) << 8) | (uint16_t)(raw_frame_buf[i+2]);

                    if (i == 0) {
                        // For the first pair of samples, don't add a leading comma
                        printf("%u, %u", sample1, sample2);
                    } else {
                        // For subsequent pairs, add a leading comma and space for parsing
                        printf(", %u, %u", sample1, sample2);
                    }
                }
                printf("]\n");
                fflush(stdout);
            } else {
                ESP_LOGE(TAG, "Frame %lu was not fully collected (collected %lu/%lu bytes).",
                         total_frames_collected_count, bytes_collected, raw_buffer_size);
            }
            ESP_LOGI(TAG, "Frame acquisition cycle complete.");
        }
    }
    free(raw_frame_buf);
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t ret;
    const int spi_max_transfer_sz = FIFO_READ_CHUNK_BYTES + 4;

    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN, .mosi_io_num = SPI_MOSI_PIN, .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = spi_max_transfer_sz
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret)); return; }

    spi_device_interface_config_t dev_config = {
        .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000,
        .mode = 0, .spics_io_num = SPI_CS_PIN, .queue_size = 7,
    };
    ret = xensiv_bgt60tr13c_init(SPI2_HOST, &dev_config);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "xensiv_bgt60tr13c_init failed: %s.", esp_err_to_name(ret)); return; }

    ESP_LOGI(TAG, "Configuring radar with settings from bgt60tr13c_config.h");
    ret = xensiv_bgt60tr13c_configure();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "xensiv_bgt60tr13c_configure failed: %s", esp_err_to_name(ret)); return; }
    
    // Set FIFO interrupt to trigger at our chunk size (4096 words)
    uint32_t sfctl_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_SFCTL);
    uint32_t new_sfctl_data = (sfctl_val & ~0x1FFFU) | 4096;
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, new_sfctl_data, true);
    ESP_LOGI(TAG, "SFCTL:FIFO_CREF set to %d words.", 4096);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary();
    xFrameTriggerSemaphore = xSemaphoreCreateBinary();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);

    ESP_LOGI(TAG, "Creating radar task");
    xTaskCreate(xensiv_bgt60tr13c_radar_task, "radar-task", 8192 * 2, NULL, 5, NULL);

    ESP_LOGI(TAG, "app_main: Initial delay...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "app_main: Starting frame triggering loop.");
    for (;;) {
        static int frame_trigger_count_main = 0;
        frame_trigger_count_main++;
        ESP_LOGI(TAG, "app_main: Triggering frame capture #%d", frame_trigger_count_main);
        xSemaphoreGive(xFrameTriggerSemaphore);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}