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
#include "radar_acquisition.h"
#include "i2c_slave.h"

static const char *TAG = "RADAR_MAIN";

#if I2C_TEST_MODE_ENABLED == 1
void i2c_test_mode_task(void *pvParameters) {
    ESP_LOGI(TAG, "I2C Test Mode Task Started. Simulating presence events.");
    bool simulated_presence = false;
    for (;;) {
        // In test mode, we toggle the raw 'presence_confirmed' state.
        // The occupancy task will then apply the time delay logic to this.
        simulated_presence = !simulated_presence;
        radar_config_set_presence_confirmed(simulated_presence);
        radar_config_set_motion_detected(simulated_presence);
        ESP_LOGI(TAG, "Test Mode: Simulated presence set to %s", simulated_presence ? "CONFIRMED" : "NOT CONFIRMED");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif


void app_main(void) {
    ESP_LOGI(TAG, "=== BGT60TR13C Radar System Starting ===");
    
    // Initialize configuration defaults
    radar_config_init();
    
    // Initialize the GPIO pin for interrupting the I2C master
    ESP_ERROR_CHECK(radar_config_init_interrupt_pin());

    // Start the task that manages the occupancy state and timer
    occupancy_management_task_create();
    
    #if I2C_TEST_MODE_ENABLED == 1
        ESP_LOGI(TAG, "=== I2C TEST MODE ENABLED ===");
        ESP_LOGI(TAG, "Radar SPI and real processing are disabled.");
        i2c_slave_task_create();
        xTaskCreate(i2c_test_mode_task, "i2c_test_task", 2048, NULL, 5, NULL);
        ESP_LOGI(TAG, "Test Mode Initialization Complete. Halting main task.");
        // The occupancy task will continue to run in the background.
        for (;;) { vTaskDelay(portMAX_DELAY); }
        return;
    #endif

    ESP_LOGI(TAG, "=== NORMAL RADAR MODE ===");
    
    radar_config_t *config = radar_config_get();
    
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN, 
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1, 
        .max_transfer_sz = 16384
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 20*1000*1000, 
        .mode = 0, 
        .spics_io_num = SPI_CS_PIN, 
        .queue_size = 7, 
    };
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_init(SPI2_HOST, &dev_config)); 
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_configure()); 
    
    ESP_LOGI(TAG, "Applying hardware settings...");

    // Set FIFO_CREF to 32 samples for smaller, more frequent IRQs.
    ESP_LOGI(TAG, "Modifying SFCTL:FIFO_CREF for 32-sample IRQ threshold.");
    uint32_t sfctl_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_SFCTL);
    uint32_t new_sfctl_data = (sfctl_val & ~0x1FFFU) | 0x020U;  // 0x20 = 32 samples
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, new_sfctl_data, true));

    // Set MAX_FRAME_CNT to 1 for single-shot frame capture.
    // This stops the radar after one frame, preventing continuous data streaming.
    ESP_LOGI(TAG, "Setting CCR2:MAX_FRAME_CNT for single-frame capture.");
    uint32_t ccr2_val_read = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    uint32_t ccr2_to_write = (ccr2_val_read & 0xFFFFF000) | 0x00000001;
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_CCR2, ccr2_to_write, true));

    ESP_LOGI(TAG, "=== Radar System Configuration Loaded ===");
    ESP_LOGI(TAG, "Frame delay set to: %lu ms", config->frame_delay_ms);
    ESP_LOGI(TAG, "===================================");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN), 
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_POSEDGE 
    };
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    
    radar_acquisition_task_create();
    i2c_slave_task_create();
    
    ESP_LOGI(TAG, "=== System Initialization Complete. Starting Main Control Loop. ===");
    
    uint32_t frame_trigger_count_main = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        if (radar_acquisition_wait_for_ready(portMAX_DELAY) == pdTRUE) {
            frame_trigger_count_main++;
            ESP_LOGD(TAG, "Main Loop: Triggering frame #%lu", frame_trigger_count_main);
            radar_acquisition_trigger_frame();

            const TickType_t xFrequency = pdMS_TO_TICKS(config->frame_delay_ms);
            
            if (xFrequency > 0) {
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
            } else {
                taskYIELD();
            }
        }
    }
}
