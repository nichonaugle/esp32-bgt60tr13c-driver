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
#define SPI_CLK_SPEED   1        // SPI Clock speed (MHz), 1 MHz in this case

#define SPI_CS_PIN  GPIO_NUM_4          // Chip Select pin for the device
#define SPI_SCK_PIN GPIO_NUM_5          // SPI Clock pin
#define SPI_MOSI_PIN GPIO_NUM_6         // SPI MOSI pin
#define SPI_MISO_PIN GPIO_NUM_7         // SPI MISO pin
#define RADAR_RESET_PIN GPIO_NUM_15     // Radar Reset pin active low

void app_main(void) {
    // SPI bus configuration
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,   // MISO Pin
        .mosi_io_num = SPI_MOSI_PIN,   // MOSI Pin
        .sclk_io_num = SPI_SCK_PIN,    // Clock Pin
        .quadwp_io_num = -1,           // Not used
        .quadhd_io_num = -1,           // Not used
        .max_transfer_sz = 128
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, 0); // 1 = DMA channel
    assert(ret == ESP_OK);

    // Customize SPI bus for the radar
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI mode 0
        .duty_cycle_pos = 128,
        .spics_io_num = SPI_CS_PIN,         // CS pin
        .queue_size = 1,                    // Transaction queue size
    };

    ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
    assert(ret == ESP_OK);
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