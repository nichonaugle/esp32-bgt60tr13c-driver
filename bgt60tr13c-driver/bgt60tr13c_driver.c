#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "bgt60tr13c-driver";

esp_spi_config_t esp_spi_config;

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config) {
    if (dev_config == NULL) {
        ESP_LOGE(TAG, "Failed to init BGT60TR13C: SPI config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Local storage of the SPI settings for use by the radar */
    esp_spi_config.spi_host = spi_host;
    esp_spi_config.dev_config = *dev_config;
    esp_spi_config.spi_handle = NULL;  // assigned once added to bus

    /* attach the device to the bus */
    esp_err_t ret = spi_bus_add_device(esp_spi_config.spi_host, &esp_spi_config.dev_config, &esp_spi_config.spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    ESP_LOGI(TAG, "BGT60TR13C added to SPI bus: %d", (int)spi_host);

    /* Set Speed */
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, 0U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SPI speed");
        return ret;
    } 

    /* Read chipid and verify */
    uint32_t chip_id = 0;
    ret = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get chip ID");
        return ret;
    }
    uint32_t chip_id_digital = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_MSK) >>
                               XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_POS;
    uint32_t chip_id_rf = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_MSK) >>
                          XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_POS;

    if ((chip_id_digital == 3U) && (chip_id_rf == 3U)) {
        ESP_LOGI(TAG, "BGT60TR13C Verified. Chip ID: %lu", chip_id);
    } else {
        ESP_LOGE(TAG, "BGT60TR13C Verification failed. Chip ID: %lu", chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    /* Soft Reset Internals 
    ret = xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_SW);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft Reset Failed");
        return ret;
    } */
    return ESP_OK;

    /* TODO: CONFIGURE SPI SETTINGS FOR FIFO R/W Operations*/
}

esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data_to_send) {
    esp_err_t ret;
    uint32_t temp;
    uint32_t reversed_data_to_send = 0;

    // Prepare the data by shifting and masking
    temp = (reg_addr << XENSIV_BGT60TR13C_SPI_REGADR_POS) & XENSIV_BGT60TR13C_SPI_REGADR_MSK;
    temp |= XENSIV_BGT60TR13C_SPI_WR_OP_MSK;  // Set write operation mask
    temp |= (data_to_send << XENSIV_BGT60TR13C_SPI_DATA_POS) & XENSIV_BGT60TR13C_SPI_DATA_MSK;

    // Reverse the word (endianess adjustment for radar)
    reversed_data_to_send |= (temp & 0x000000FF) << 24;
    reversed_data_to_send |= (temp & 0x0000FF00) << 8;
    reversed_data_to_send |= (temp & 0x00FF0000) >> 8;
    reversed_data_to_send |= (temp & 0xFF000000) >> 24;

    spi_transaction_t trans = {
        .length = 32,    // Data length in bits (8 bits for one byte per array entry)
        .tx_buffer = &reversed_data_to_send,        // Pointer to data to send
        .rx_buffer = NULL,                // Pointer to buffer to receive data (set NULL if no response)
    };

    ret = spi_device_transmit(esp_spi_config.spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed");
        return ret;
    }
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr, uint32_t* data_to_recieve) {
    if (data_to_recieve == NULL) {
        ESP_LOGE(TAG, "Failed to get register from BGT60TR13C: Recieving data buffer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx_data[4];
    uint8_t rx_data[4] = {0};

    // Prepare the data by shifting and masking
    uint32_t temp = (reg_addr << XENSIV_BGT60TR13C_SPI_REGADR_POS) & XENSIV_BGT60TR13C_SPI_REGADR_MSK;

    // Reverse the word (endianess adjustment for radar)
    tx_data[0] = (temp >> 24) & 0xFF;
    tx_data[1] = (temp >> 16) & 0xFF;
    tx_data[2] = (temp >> 8) & 0xFF;
    tx_data[3] = temp & 0xFF;

    ESP_LOGI(TAG, "Received data: %d", tx_data[0]); // 0000 0100
    ESP_LOGI(TAG, "Received data: %d", tx_data[1]); // 0000 0000
    ESP_LOGI(TAG, "Received data: %d", tx_data[2]); // 0000 0000
    ESP_LOGI(TAG, "Received data: %d", tx_data[3]); // 0000 0000

    spi_transaction_t trans = {
        .length = 8 * sizeof(tx_data),    // Data length in bits (8 bits for one byte per array entry)
        .tx_buffer = tx_data,             // Pointer to data to send
        .rxlength = 8 * sizeof(rx_data),
        .rx_buffer = rx_data,             // Pointer to buffer to receive data (set NULL if no response)
        .flags = 0,                       // Normal SPI (full-duplex)
    };

    esp_err_t ret = spi_device_polling_transmit(esp_spi_config.spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed");
        return ret;
    }

    temp = 0;  // Clears before use
    temp = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    ESP_LOGI(TAG, "Received data: %d", rx_data[0]); // 0000 0000
    ESP_LOGI(TAG, "Received data: %d", rx_data[1]); // 1010 1010
    ESP_LOGI(TAG, "Received data: %d", rx_data[2]); // 1100 1001
    ESP_LOGI(TAG, "Received data: %d", rx_data[3]); // 0011 1111

    // Reverse the recieved word (endianess adjustment for radar)

    *data_to_recieve = (((temp & 0x000000FF) << 24) | ((temp & 0x0000FF00) << 8) | ((temp & 0x00FF0000) >> 8) | ((temp & 0xFF000000) >> 24)) & XENSIV_BGT60TR13C_SPI_DATA_MSK;

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type) {
    uint32_t tmp;
    int32_t status;

    status = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN, &tmp);
    if (status == ESP_OK)
    {
        tmp |= (uint32_t)reset_type;
        status = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp);
    }

    uint32_t timeout = XENSIV_BGT60TR13C_RESET_WAIT_TIMEOUT;
    if (status == ESP_OK)
    {
        while (timeout > 0U)
        {
            status = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN, &tmp);
            if ((status == ESP_OK) && ((tmp & (uint32_t)reset_type) == 0U))
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