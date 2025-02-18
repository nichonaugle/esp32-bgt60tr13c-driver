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
    uint16_t *rx_frame_buf = (uint16_t *)malloc(frame_size * sizeof(uint16_t));
    if (rx_frame_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }
    memset(rx_frame_buf, 0, frame_size * sizeof(uint16_t));

    /* Start a frame collection (only one) */
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_start_frame_capture());

    /* Collect Frames Till Complete */
    for(;;) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "reading fifo");
            xensiv_bgt60tr13c_fifo_read(rx_frame_buf, irq_frame_size);
        }
    }

    /* Free the allocated buffers */
    free(rx_frame_buf);
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