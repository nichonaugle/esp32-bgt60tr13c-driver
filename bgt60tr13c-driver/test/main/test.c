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

static const char * TAG = "spi-test-runner";

#define SPI_HOST    SPI2_HOST          // Choose SPI2, or SPI1_HOST or SPI3_HOST
#define SPI_CLK_SPEED   1        // SPI Clock speed (MHz), 1 MHz in this case

#define SPI_CS_PIN  GPIO_NUM_18          // Chip Select pin for the device
#define SPI_SCK_PIN GPIO_NUM_6          // SPI Clock pin
#define SPI_MOSI_PIN GPIO_NUM_2         // SPI MOSI pin
#define SPI_MISO_PIN GPIO_NUM_7         // SPI MISO pin
#define RADAR_RESET_PIN GPIO_NUM_10     // Radar Reset pin active low

static void cs_high(spi_transaction_t* t)
{
    gpio_set_level(SPI_CS_PIN, 1);
    ESP_EARLY_LOGV(TAG, "cs high %d.", (int)SPI_CS_PIN);
}

static void cs_low(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs low %d.", (int)SPI_CS_PIN);
    gpio_set_level(SPI_CS_PIN, 0);
}

void app_main(void) {
    // SPI bus configuration
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
    /*
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SPI_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(SPI_CS_PIN, 1);*/

    // Customize SPI bus for the radar
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000, // 1 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = SPI_CS_PIN,         // CS pin
        .queue_size = 1,                    // Transaction queue size
        //.pre_cb = cs_low,
        //.post_cb = cs_high
    };

    spi_device_handle_t spi;

    ret = spi_bus_add_device(SPI_HOST, &dev_config, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Successfully added SPI device!");

    uint8_t tx_buffer[4] = {0};

    // Prepare the data by shifting and masking
    tx_buffer[0] = (0x02U << 1) | 0x00; // R/W bit set to 0 for read

    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .rxlength = 8 * sizeof(tx_buffer),
        .flags     = SPI_TRANS_USE_RXDATA
    };

    ret = spi_device_polling_transmit(spi, &t);  // Transmit!
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed: %s", esp_err_to_name(ret));
        return;
    }

    // Combine the received bytes into a 24-bit value
    ESP_LOGI(TAG, "Transmitted Data: %lu", (uint32_t)((tx_buffer[0] << 24) | (tx_buffer[1] << 16) | (tx_buffer[2] << 8) | tx_buffer[3]));
    ESP_LOGI(TAG, "Received Data: %lu", (uint32_t)((t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3]));
}






/* Configure RADAR_RESET_PIN as output with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(RADAR_RESET_PIN, 1);  */