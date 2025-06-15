#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Project includes
#include "bgt60tr13c_driver.h"
#include "radar_config.h"
#include "radar_proc/radar_acquisition.h"
#include "i2c/i2c_slave.h"

static const char *TAG = "RADAR_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "=== BGT60TR13C Radar System Starting ===");
    
    // Initialize radar configuration first
    radar_config_init();
    radar_config_t *config = radar_config_get();
    
    // Initialize SPI bus for radar communication
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN, 
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1, 
        .max_transfer_sz = 4096 
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    
    // Initialize radar device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 20*1000*1000, 
        .mode = 0, 
        .spics_io_num = SPI_CS_PIN, 
        .queue_size = 7, 
    };
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_init(SPI2_HOST, &dev_config)); 
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_configure()); 
    
    // Get frame configuration for logging
    uint32_t frame_size_samples, irq_frame_size;
    get_frame_size(&frame_size_samples);
    get_interrupt_frame_size_trigger(&irq_frame_size);
    
    // Print radar system configuration
    ESP_LOGI(TAG, "=== Radar System Configuration ===");
    ESP_LOGI(TAG, "Frame size: %lu samples", frame_size_samples);
    ESP_LOGI(TAG, "IRQ frame size: %lu bytes", irq_frame_size);
    ESP_LOGI(TAG, "Base range resolution: %.2f cm", RANGE_RESOLUTION_M * 100);
    ESP_LOGI(TAG, "Effective range resolution: %.2f cm (4x interpolated)", (RANGE_RESOLUTION_M / 4.0f) * 100);
    ESP_LOGI(TAG, "FFT size: %d (4x zero-padded)", RANGE_FFT_SIZE);
    ESP_LOGI(TAG, "Total range bins: %d", N_RANGE_BINS);
    ESP_LOGI(TAG, "Theoretical max range: %.2f m", R_MAX_M);
    ESP_LOGI(TAG, "Current Configuration:");
    ESP_LOGI(TAG, "  - Antenna index: %d", config->antenna_index);
    ESP_LOGI(TAG, "  - Useful range: %.1f m", config->useful_range_m);
    ESP_LOGI(TAG, "  - Useful range bins: %d", (int)(config->useful_range_m / (RANGE_RESOLUTION_M / 4.0f)));
    ESP_LOGI(TAG, "  - CFAR - Guard: %d, Ref: %d, Bias: %.1f dB", 
             config->num_guard_cells, config->num_ref_cells, config->cfar_bias_db);
    ESP_LOGI(TAG, "  - CFAR starts at bin %d (%.3f m)", 
             config->num_guard_cells + config->num_ref_cells, 
             (config->num_guard_cells + config->num_ref_cells) * (RANGE_RESOLUTION_M / 4.0f));
    ESP_LOGI(TAG, "  - Windowing: Range Hamming (%d samples) + Doppler Hamming (%d chirps)", 
             N_SAMPLES_PER_CHIRP, M_CHIRPS);
    ESP_LOGI(TAG, "  - UART plotting: %s", config->enable_uart_plotting ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "  - Frame delay: %lu ms", config->frame_delay_ms);
    ESP_LOGI(TAG, "===================================");
    
    // Configure GPIO for radar IRQ
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN), 
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_POSEDGE 
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    
    // Create radar acquisition task
    radar_acquisition_task_create();
    
    // Create I2C slave task
    i2c_slave_task_create();
    
    ESP_LOGI(TAG, "=== System Initialization Complete ===");
    ESP_LOGI(TAG, "Radar acquisition task started");
    ESP_LOGI(TAG, "System ready for operation");
    
    // Main loop - just delay to keep the task running
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}