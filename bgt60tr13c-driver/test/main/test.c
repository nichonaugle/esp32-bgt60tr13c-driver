#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "bgt60tr13c_driver.h"

static const char * TAG = "spi-test-runner";

#define SPI_HOST    SPI2_HOST          // Choose SPI2, or SPI1_HOST or SPI3_HOST
#define SPI_CLK_SPEED   1000000        // SPI Clock speed, 10 MHz in this case

#define SPI_CS_PIN  GPIO_NUM_4       // Chip Select pin for the device
#define SPI_SCK_PIN GPIO_NUM_5      // SPI Clock pin
#define SPI_MOSI_PIN GPIO_NUM_6     // SPI MOSI pin
#define SPI_MISO_PIN GPIO_NUM_7     // SPI MISO pin

void app_main(void) {
    // SPI bus configuration
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,   // MISO Pin
        .mosi_io_num = SPI_MOSI_PIN,   // MOSI Pin
        .sclk_io_num = SPI_SCK_PIN,    // Clock Pin
        .quadwp_io_num = -1,           // Not used
        .quadhd_io_num = -1            // Not used
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, 0); // 1 = DMA channel
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
    }
    ESP_LOGI(TAG, "Successfully initialized bus!");

    // Customize SPI bus for the radar
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = SPI_CLK_SPEED,       // SPI clock speed
        .mode = 0,                             // SPI mode 0: CPOL = 0, CPHA = 0
        .spics_io_num = SPI_CS_PIN,            // Chip Select pin
        .queue_size = 7,                      // Queue size for SPI transactions
        .pre_cb = NULL,                        // Optional callback before transferring data
        //.command_bits = 0,
        //.address_bits = 0,
        //.flags = 0                             // Active Low CS
    };

    ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BGT60TR13C");
    }
    ESP_LOGI(TAG, "Successfully initialized BGT60TR13C!");
}
