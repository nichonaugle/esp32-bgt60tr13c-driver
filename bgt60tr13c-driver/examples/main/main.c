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
#include "radar_acquisition.h" // Using the updated header
#include "i2c_slave.h"

static const char *TAG = "RADAR_MAIN";

// Test mode task - toggles motion detection flag periodically
void test_mode_task(void *pvParameters) {
    ESP_LOGI(TAG, "I2C Test Mode Task Started");
    
    bool motion_state = false;
    
    for (;;) {
        // Toggle motion detection every 3 seconds
        motion_state = !motion_state;
        radar_config_set_motion_detected(motion_state);
        
        ESP_LOGI(TAG, "Test Mode: Motion detected = %s", motion_state ? "TRUE" : "FALSE");
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== BGT60TR13C Radar System Starting ===");
    
    // Initialize radar configuration first
    radar_config_init();
    radar_config_t *config = radar_config_get();
    
    // Initialize I2C master interrupt pin
    esp_err_t interrupt_err = radar_config_init_interrupt_pin();
    if (interrupt_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize I2C master interrupt pin: %s", esp_err_to_name(interrupt_err));
    }
    
    // Check if test mode is enabled (compile-time define)
    if (I2C_TEST_MODE_ENABLED) {
        ESP_LOGI(TAG, "=== I2C TEST MODE ENABLED ===");
        ESP_LOGI(TAG, "Radar SPI and processing disabled");
        ESP_LOGI(TAG, "Only I2C slave and test task will run");
        
        i2c_slave_task_create();
        xTaskCreate(test_mode_task, "test_mode_task", 4096, NULL, 3, NULL);
        
        ESP_LOGI(TAG, "=== Test Mode Initialization Complete ===");
        // In test mode, we just loop forever and do nothing in main.
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        return;
    }
    
    // Normal radar mode - proceed with full initialization
    ESP_LOGI(TAG, "=== NORMAL RADAR MODE ===");
    
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
    
    // Print radar system configuration (this section is unchanged)
    ESP_LOGI(TAG, "=== Radar System Configuration Loaded ===");
    // ... you could add detailed printouts here if desired ...
    ESP_LOGI(TAG, "Frame delay set to: %lu ms", config->frame_delay_ms);
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
    
    // Install GPIO ISR service and add our handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    
    // Create the subordinate tasks
    radar_acquisition_task_create();
    i2c_slave_task_create();
    
    ESP_LOGI(TAG, "=== System Initialization Complete. Starting Main Control Loop. ===");
    
    // ===================================================================
    // ===         NEW SYNCHRONIZED FRAME TRIGGER LOOP                 ===
    // ===================================================================
    uint32_t frame_trigger_count_main = 0;
    for (;;) {
        // 1. Wait until the radar acquisition task signals it's ready for a new frame.
        //    This call will block until the previous frame is fully processed.
        if (radar_acquisition_wait_for_ready(portMAX_DELAY) == pdTRUE) {
            
            // 2. The acquisition task is ready. Now we can enforce the desired
            //    frame rate by delaying *before* triggering the next one.
            vTaskDelay(pdMS_TO_TICKS(config->frame_delay_ms));

            // 3. Trigger the next frame capture.
            frame_trigger_count_main++;
            ESP_LOGI(TAG, "Main Loop: Radar task is ready. Triggering frame #%lu", frame_trigger_count_main);
            radar_acquisition_trigger_frame();
        }
    }
}