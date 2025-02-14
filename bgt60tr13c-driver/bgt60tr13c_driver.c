#include <string.h>
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "bgt60tr13c_config.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char* TAG = "bgt60tr13c-driver";
spi_device_handle_t spi;
SemaphoreHandle_t xSemaphore = NULL;

// init memory allocated to the recieved frame defined by [ samples * chirps * rx_antennas(3) ]
static const uint32_t frame_size = XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP * 
                                    XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME * 
                                    XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS;

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config, gpio_num_t interrupt_pin) {
    assert(dev_config != NULL);
    
    /* attach the device to the bus */
    esp_err_t ret = spi_bus_add_device(spi_host, dev_config, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    ESP_LOGI(TAG, "BGT60TR13C added to SPI bus: %d", (int)spi_host);

    /* FIFO Settings; Interrupt is triggered when any data enters FIFO > 0 */
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, 0U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SPI speed");
        return ret;
    } 

    /* Read chipid and verify */
    uint32_t chip_id = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CHIP_ID);

    uint32_t chip_id_digital = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_MSK) >>
                               XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_POS;
    uint32_t chip_id_rf = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_MSK) >>
                          XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_POS;

    if ((chip_id_digital == 3U) && (chip_id_rf == 3U)) {
        ESP_LOGI(TAG, "BGT60TR13C Verified. Digital Chip ID: %lu RF Chip ID: %lu", chip_id_digital, chip_id_rf);
    } else {
        ESP_LOGE(TAG, "BGT60TR13C Verification failed. Returned Chip ID: %lu", chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    /* Soft Reset Internals */
    ret = xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_SW);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft Reset Failed");
        return ret;
    }

    /* General settings from bgt60tr13c_config.h ; Interrupt is triggered when FIFO > 2048 bits ; HS mode off*/
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) {
        uint32_t val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t reg_data = ((val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                XENSIV_BGT60TR13C_SPI_DATA_POS);

        ret = xensiv_bgt60tr13c_set_reg(reg_addr, reg_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set register:%lu to value:%lu", reg_addr, reg_data);
            return ret;
        } else {
            ESP_LOGI(TAG, "Success. Set register:%lu to value:%lu", reg_addr, reg_data);
        }
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << interrupt_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on falling edge
    };
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        vTaskDelete(NULL);
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(interrupt_pin, gpio_radar_isr_handler, NULL);

    return ret;
}

void IRAM_ATTR gpio_radar_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void xensiv_bgt60tr13c_radar_task(void *pvParameters) {
    ESP_LOGI(TAG, "starting radar task");
    uint32_t running_buf_idx = 0;
    uint32_t rx_buf_size = 0;
    uint32_t *frame_buf = (uint32_t *)malloc(frame_size * sizeof(uint32_t));
    uint32_t *temp_rx_buf = (uint32_t *)malloc(frame_size * sizeof(uint32_t));
    if (frame_buf == NULL || temp_rx_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }
    memset(frame_buf, 0, frame_size * sizeof(uint32_t));
    memset(temp_rx_buf, 0, frame_size * sizeof(uint32_t));
    
    uint32_t data = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
    data |= XENSIV_BGT60TR13C_REG_MAIN_FRAME_START_MSK;
    assert(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, data) == ESP_OK);

    for(;;) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "reading fifo");
            xensiv_bgt60tr13c_fifo_read(temp_rx_buf, XENSIV_BGT60TR13C_IRQ_TRIGGER_FRAME_SIZE);
            for (uint16_t i = 0; i < XENSIV_BGT60TR13C_IRQ_TRIGGER_FRAME_SIZE; i++) {
                if (running_buf_idx == frame_size) {
                    char log_buffer[frame_size];  // Adjust size based on expected array length
                    int offset = 0;

                    offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "[");

                    for (uint32_t i = 0; i < frame_size; i++) {
                        if (offset < sizeof(log_buffer) - 20) {  // Prevent buffer overflow
                            offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "%lu", frame_buf[i]);
                            if (i < frame_size - 1) {
                                offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, ", ");
                            }
                        } else {
                            break;  // Stop writing if buffer is full
                        }
                    }

                    snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "]\n");
                    ESP_LOGI(TAG, "%s", log_buffer); 
                    /* 
                    printf("[");
                    for(uint32_t i = 0; i < frame_size; i++) {
                        printf("%lu", frame_buf[i]);
                        if (i < frame_size - 1) {
                            printf(", ");
                        }
                    }
                    printf("]\n");*/
                    memset(frame_buf, 0, frame_size * sizeof(uint32_t));
                    running_buf_idx = 0;
                }
                frame_buf[running_buf_idx] = temp_rx_buf[i];
                running_buf_idx++;
            }
    
            memset(temp_rx_buf, 0, frame_size * sizeof(uint32_t));
            //xSemaphoreGive(xSemaphore);
        }
    }

    // Free the allocated buffers
    free(frame_buf);
    free(temp_rx_buf);
}

void start_bgt60tr13c_task() {
    printf("starting radar task");
    xTaskCreate(xensiv_bgt60tr13c_radar_task, "radar-task", 4096, NULL, 10, NULL);
}

esp_err_t xensiv_bgt60tr13c_fifo_read(uint32_t *frame_buf, uint32_t rx_buf_size) {
    esp_err_t ret;

    // Aquire SPI bus
    ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE("FIFO", "Failed to acquire SPI bus");
        return ret;
    }

    uint8_t tx_data[4] = {0};

    // Send Burst Read Command (based on number of frames to read divided by word size)
    uint32_t data = XENSIV_BGT60TR13C_SPI_BURST_MODE_CMD |
                        (XENSIV_BGT60TR13C_REG_FIFO_TR13C << XENSIV_BGT60TR13C_SPI_BURST_MODE_SADR_POS) | 
                        (rx_buf_size/32 << XENSIV_BGT60TR13C_SPI_BURST_MODE_LEN_POS);

    tx_data[0] = (data >> 24) & 0xFF;
    tx_data[1] = (data >> 16) & 0xFF;
    tx_data[2] = (data >> 8) & 0xFF;
    tx_data[3] = (data) & 0xFF;

    // Create SPI transaction frame
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * rx_buf_size,
        .tx_buffer = tx_data,
        .rxlength = 8 * rx_buf_size,
        .rx_buffer = frame_buf,
        .flags = 0
    };

    // Perform the transaction
    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE("FIFO", "SPI transaction failed");
        spi_device_release_bus(spi);
        return ret;
    }
    // De-aquire bus
    spi_device_release_bus(spi);
    return ret;
}

esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data) {
    esp_err_t ret;
    uint8_t tx_buffer[4];

    // Prepare the command byte (7-bit address + R/W bit)
    tx_buffer[0] = (reg_addr << 1) | 0x01; // R/W bit set to 1 for write

    // Prepare the data bytes (24-bit data)
    tx_buffer[1] = (data >> 16) & 0xFF;
    tx_buffer[2] = (data >> 8) & 0xFF;
    tx_buffer[3] = data & 0xFF;

    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer
    };
    
    ret = spi_device_polling_transmit(spi, &t);  // Transmit!
    assert(ret == ESP_OK);               // Should have no issues

    return ret;
}

uint32_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr) {
    esp_err_t ret;
    uint8_t tx_buffer[4] = {0};

    tx_buffer[0] = (reg_addr << 1) | 0x00; // R/W bit set to 0 for read

    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .rxlength = 8 * sizeof(tx_buffer),
        .flags     = SPI_TRANS_USE_RXDATA
    };

    ret = spi_device_polling_transmit(spi, &t);  // Transmit!
    assert(ret == ESP_OK);               // Should have no issues

    return (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];
}

esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type) {
    uint32_t tmp;
    int32_t status;

    tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
    tmp |= (uint32_t)reset_type;
    status = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp);

    uint32_t timeout = XENSIV_BGT60TR13C_RESET_WAIT_TIMEOUT;
    if (status == ESP_OK)
    {
        while (timeout > 0U)
        {
            tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
            if (((tmp & (uint32_t)reset_type) == 0U))
            {
                break;
            }
            --timeout;
        }
    }

    if (status == ESP_OK)
    {
        if (timeout == 0U)
        {
            return ESP_ERR_TIMEOUT;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(XENSIV_BGT60TR13C_SOFT_RESET_DELAY_MS));
        }
    }
    return ESP_OK;
}