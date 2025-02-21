#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "freertos/semphr.h"

static const char * TAG = "spi-test-runner";

#define SPI_HOST    SPI2_HOST           // Choose SPI2, or SPI1_HOST or SPI3_HOST
#define SPI_CLK_SPEED   20              // SPI Clock speed (MHz), 20 MHz in this case

#define SPI_CS_PIN  GPIO_NUM_13          // Chip Select pin for the device
#define SPI_SCK_PIN GPIO_NUM_27          // SPI Clock pin
#define SPI_MOSI_PIN GPIO_NUM_25         // SPI MOSI pin
#define SPI_MISO_PIN GPIO_NUM_26         // SPI MISO pin
#define RADAR_IRQ_PIN GPIO_NUM_4         // Radar Reset pin active low
#define RADAR_RESET_PIN GPIO_NUM_14      // Radar Reset pin active low

SemaphoreHandle_t xSemaphore = NULL;

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void xensiv_bgt60tr13c_radar_task(void *pvParameters) {
    /* Collects one frame when triggered */
    ESP_LOGI(TAG, "starting radar task");

    /* Variable initialization */
    uint32_t frame_size = 0;
    uint32_t irq_frame_size = 0;
    ESP_ERROR_CHECK(get_frame_size(&frame_size));
    ESP_ERROR_CHECK(get_interrupt_frame_size_trigger(&irq_frame_size));

    /* Real and temp frame buffer initializtion */
    uint32_t frame_buf_len = frame_size;                        // Times three since word size is 24 bits and buffer size is in bytes
    uint32_t temp_buf_len = 1023;                               // Times three since word size is 24 bits and buffer size is in bytes
    uint16_t *frame_buf = (uint16_t *)malloc(frame_buf_len * sizeof(uint16_t));    // Stores 12-bit values
    uint8_t *temp_buf = (uint8_t *)malloc(temp_buf_len * sizeof(uint8_t));
    if (frame_buf == NULL || temp_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }
    memset(frame_buf, 0, frame_buf_len);
    memset(temp_buf, 0, temp_buf_len);

    // DEBUG REMOVE ME
    ESP_LOGI(TAG, "frame_size: %lu", frame_size);
    ESP_LOGI(TAG, "irq_frame_size: %lu", irq_frame_size);
    ESP_LOGI(TAG, "frame_buf_len: %lu", frame_buf_len);
    ESP_LOGI(TAG, "temp_buf_len: %lu", temp_buf_len);

    /* Start a frame collection (only one) */
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture());

    /* Collect Frames Till Complete */
    uint32_t current_idx = 0;
    uint32_t print_counter = 0; // Track when to print

    for (;;) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len, irq_frame_size);

            for (uint32_t idx = 0; idx < temp_buf_len && current_idx < frame_buf_len; idx += 3, current_idx += 2) {
                frame_buf[current_idx] = (temp_buf[idx] << 4) | (temp_buf[idx + 1] >> 4);
                frame_buf[current_idx + 1] = ((temp_buf[idx + 1] & 0x0F) << 8) | temp_buf[idx + 2];
            }

            memset(temp_buf, 0, temp_buf_len);

            if (current_idx >= frame_size) {
                ESP_LOGI(TAG, "Final frame collected.");
                vSemaphoreDelete(xSemaphore); // Delete semaphore when frame is complete
                printf("Frame: [");
                for (uint32_t i = 0; i < frame_size - 1; i++) {
                    printf("%d, ", frame_buf[i]);
                }
                printf("%d", frame_buf[frame_size - 1]);
                printf("]\n");
                vTaskDelay(portMAX_DELAY);
            }
        }
    }

    /* Free the allocated buffers */
    free(frame_buf);
    free(temp_buf);
}

void app_main(void) {
    // Recommended SPI bus configuration
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,   // MISO Pin
        .mosi_io_num = SPI_MOSI_PIN,   // MOSI Pin
        .sclk_io_num = SPI_SCK_PIN,    // Clock Pin
        .quadwp_io_num = -1,           // Not used
        .quadhd_io_num = -1,           // Not used
        .max_transfer_sz = 32          // Maximum transfer size
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO); // Use automatic DMA channel selection
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Successfully initialized SPI bus!");

    // Customize SPI bus for the radar
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000, // 1 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = SPI_CS_PIN,         // CS pin
        .queue_size = 1,                    // Transaction queue size
    };
    
    ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
    assert(ret == ESP_OK);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Trigger on falling edge
    };
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        vTaskDelete(NULL);
    }

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL));

    xTaskCreate(xensiv_bgt60tr13c_radar_task, "radar-task", 16384, NULL, 10, NULL);
}