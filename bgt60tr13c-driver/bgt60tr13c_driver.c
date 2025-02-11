#include <string.h>
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "bgt60tr13c-driver";

spi_device_handle_t spi;

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config) {
    assert(dev_config != NULL);
    
    /* attach the device to the bus */
    esp_err_t ret = spi_bus_add_device(spi_host, dev_config, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    ESP_LOGI(TAG, "BGT60TR13C added to SPI bus: %d", (int)spi_host);

    vTaskDelay(100/portTICK_PERIOD_MS);  // 10ms delay

    /* Set Speed */
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_SFCTL, 8192U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SPI speed");
        return ret;
    } 

    vTaskDelay(100/portTICK_PERIOD_MS);  // 10ms delay

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
    return ret;

    /* TODO: CONFIGURE SPI SETTINGS FOR FIFO R/W Operations*/
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
    ESP_LOGI(TAG, "Received Data Soft Reset: %lu", tmp); 
    tmp |= (uint32_t)reset_type;
    status = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp);

    uint32_t timeout = XENSIV_BGT60TR13C_RESET_WAIT_TIMEOUT;
    if (status == ESP_OK)
    {
        while (timeout > 0U)
        {
            tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
            ESP_LOGI(TAG, "Received Data Soft Reset in Loop: %lu", tmp); 
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