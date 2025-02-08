#include "xensiv_bgt60tr13c.h"
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "XENSIV_bgt60tr13c";

esp_spi_config_t esp_spi_config;

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t dev_config, spi_device_handle_t spi_handle) {
    if (spi_host == NULL || dev_config == NULL || spi_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Local storage of the SPI settings for use by the radar */
    esp_spi_presets_t esp_spi_config = {
        .spi_host = spi_host,
        .dev_config = dev_config,
        .spi_handle = spi_handle
    };

    /* attach the device to the bus */
    ret = spi_bus_add_device(spi_host, &dev_config, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to add SPI device");
        return;
    }
    ESP_LOGI("SPI", "BGT60TR13C added to SPI bus: %d", (int)spi_host);

    /* send commands to initialize the radar */

    // Continue with initialization logic (chip detection, etc.)

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data_to_send) {
    if (reg_addr == NULL || data_to_send == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    uint32_t temp;
    uint32_t reversed_data_to_send;

    // Prepare the data by shifting and masking
    temp = (reg_addr << XENSIV_BGT60TRXX_SPI_REGADR_POS) & XENSIV_BGT60TRXX_SPI_REGADR_MSK;
    temp |= XENSIV_BGT60TRXX_SPI_WR_OP_MSK;  // Set write operation mask
    temp |= (data_to_send << XENSIV_BGT60TRXX_SPI_DATA_POS) & XENSIV_BGT60TRXX_SPI_DATA_MSK;

    // Reverse the word (endianess adjustment for radar)
    reversed_data_to_send |= (temp & 0x000000FF) << 24;
    reversed_data_to_send |= (temp & 0x0000FF00) << 8;
    reversed_data_to_send |= (temp & 0x00FF0000) >> 8;
    reversed_data_to_send |= (temp & 0xFF000000) >> 24;

    spi_transaction_t trans = {
        .length = sizeof(uint32_t),    // Data length in bits (8 bits for one byte per array entry)
        .tx_buffer = reversed_data_to_send,        // Pointer to data to send
        .rx_buffer = NULL,                // Pointer to buffer to receive data (set NULL if no response)
    };

    ret = spi_device_transmit(esp_spi_config.spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI transmission failed");
        return ret;
    }
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr, uint32_t* data_to_recieve) {
    if (reg_addr == NULL || data_to_recieve == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t temp;
    uint32_t reversed_data_to_send;

    // Prepare the data by shifting and masking
    temp = (reg_addr << XENSIV_BGT60TR13C_SPI_REGADR_POS) & XENSIV_BGT60TR13C_SPI_REGADR_MSK;

    // Reverse the word (endianess adjustment for radar)
    reversed_data_to_send |= (temp & 0x000000FF) << 24;
    reversed_data_to_send |= (temp & 0x0000FF00) << 8;
    reversed_data_to_send |= (temp & 0x00FF0000) >> 8;
    reversed_data_to_send |= (temp & 0xFF000000) >> 24;

    spi_transaction_t trans = {
        .length = sizeof(uint32_t),    // Data length in bits (8 bits for one byte per array entry)
        .tx_buffer = reversed_data_to_send,        // Pointer to data to send
        .rx_buffer = data_to_recieve,                // Pointer to buffer to receive data (set NULL if no response)
    };

    ret = spi_device_transmit(esp_spi_config.spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI transmission failed");
        return ret;
    }

    temp = *data_to_recieve;

    // Reverse the recieved word (endianess adjustment for radar)
    reversed_data_to_send |= (temp & 0x000000FF) << 24;
    reversed_data_to_send |= (temp & 0x0000FF00) << 8;
    reversed_data_to_send |= (temp & 0x00FF0000) >> 8;
    reversed_data_to_send |= (temp & 0xFF000000) >> 24;

    *data = reversed_data_to_send & XENSIV_BGT60TR13C_SPI_DATA_MSK;

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type) {
    uint32_t tmp;
    int32_t status;

    status = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN, &tmp);
    if (status == ESP_OK)
    {
        tmp |= (uint32_t)reset_type;
        status = xensiv_bgt60trxx_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp);
    }

    uint32_t timeout = XENSIV_BGT60TR13C_RESET_WAIT_TIMEOUT;
    if (status == ESP_OK)
    {
        while (timeout > 0U)
        {
            status = xensiv_bgt60trxx_get_reg(XENSIV_BGT60TR13C_REG_MAIN, &tmp);
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

// Theres no way this function works. Fix it later
esp_err_t xensiv_bgt60tr13c_get_fifo_data(uint16_t* data, uint32_t num_samples) {
    // FIFO data reading implementation
    esp_err_t ret = ESP_OK;
    if (data == NULL || num_samples == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((num_samples % 2U) == 0U) {
        ESP_LOGE(TAG, "Number of samples must be even");
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t received_data;
    uint32_t reversed_data_to_send;
    uint32_t reg_addr = XENSIV_BGT60TR13C_SPI_BURST_MODE_CMD; //|
                        //(dev->type->fifo_addr << XENSIV_BGT60TR13C_SPI_BURST_MODE_SADR_POS);

    reversed_data_to_send |= (reg_addr & 0x000000FF) << 24;
    reversed_data_to_send |= (reg_addr & 0x0000FF00) << 8;
    reversed_data_to_send |= (reg_addr & 0x00FF0000) >> 8;
    reversed_data_to_send |= (reg_addr & 0xFF000000) >> 24;

    /* SPI read burst mode command */
    ret = xensiv_bgt60tr13c_get_reg(reversed_data_to_send, &received_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SPI burst command");
        return ret;
    }

    // Check the received GSR0 status (error check)
    if ((received_data & (XENSIV_BGT60TR13C_REG_GSR0_FOU_ERR_MSK |
                           XENSIV_BGT60TR13C_REG_GSR0_SPI_BURST_ERR_MSK |
                           XENSIV_BGT60TR13C_REG_GSR0_CLK_NUM_ERR_MSK)) != 0) {
        ESP_LOGE(TAG, "FIFO read error or burst mode error");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_start_frame(const xensiv_bgt60tr13c_t* dev, bool start) {
    // Start or stop frame logic
    return ESP_OK;
}
