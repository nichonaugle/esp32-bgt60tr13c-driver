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

esp_err_t xensiv_bgt60tr13c_soft_reset(const xensiv_bgt60tr13c_t* dev, xensiv_bgt60tr13c_reset_t reset_type) {
    // Reset logic
    return ESP_OK;
}

uint16_t xensiv_bgt60tr13c_get_fifo_size(const xensiv_bgt60tr13c_t* dev) {
    return dev->type->fifo_size;
}

esp_err_t xensiv_bgt60tr13c_get_fifo_data(const xensiv_bgt60tr13c_t* dev, uint16_t* data, uint32_t num_samples) {
    // FIFO data reading implementation
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_start_frame(const xensiv_bgt60tr13c_t* dev, bool start) {
    // Start or stop frame logic
    return ESP_OK;
}
