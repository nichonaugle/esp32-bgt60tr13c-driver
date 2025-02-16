#include <string.h>
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "bgt60tr13c_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "bgt60tr13c-driver";
spi_device_handle_t spi;
bool radar_configured = false;

/* Determine size of recieved frame defined by [ samples * chirps * rx_antennas(3) ] */
static uint32_t frame_size = XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP * 
                                    XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME * 
                                    XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS;

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config) {
    /* Ensure the dev_config is has been configured */
    assert(dev_config != NULL);
    
    /* attach the device to the spi bus */
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, dev_config, &spi));

    /* Read chipid and verify that it is properly connected */
    uint32_t chip_id = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CHIP_ID);

    uint32_t chip_id_digital = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_MSK) >>
                               XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_POS;
    uint32_t chip_id_rf = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_MSK) >>
                          XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_POS;

    if ((chip_id_digital == 3U) && (chip_id_rf == 3U)) {
        ESP_LOGI(TAG, "BGT60TR13C Verified. Digital Chip ID: %lu RF Chip ID: %lu", chip_id_digital, chip_id_rf);
    } else {
        ESP_LOGE(TAG, "BGT60TR13C Verification failed. Returned Chip ID: %lu. Make sure radar is properly connected.", chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    /* Soft Reset Internals */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_SW), TAG, "Failed to soft reset radar SW");

    /* General settings from bgt60tr13c_config.h ; Interrupt is triggered when FIFO > 2048 bits ; HS mode off */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_configure(), TAG, "Failed to configure radar registers");

    /* Bypass FIFO error notification while configuring */
    radar_configured = true;

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_configure() {
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) {
        uint32_t val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t reg_data = ((val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                XENSIV_BGT60TR13C_SPI_DATA_POS);

        xensiv_bgt60tr13c_set_reg(reg_addr, reg_data, true);
    }
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_start_frame_capture() {
    /* Gets MAIN register data, ensuring nothing is overwritten */
    uint32_t tx_data = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);

    /* Masks with command to start frame capture */
    tx_data |= XENSIV_BGT60TR13C_REG_MAIN_FRAME_START_MSK;

    /* Masks with command to start frame capture */
    tx_data |= XENSIV_BGT60TR13C_REG_MAIN_FRAME_START_MSK;
    
    /* Sets register to start frame capture. verification is false since it is a write only bit in the register */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tx_data, false), TAG, "Failed to start radar frame capture");

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_fifo_read(uint16_t *frame_buf, uint32_t rx_buf_size) {
    esp_err_t ret;

    /* Aquire SPI bus */
    ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE("FIFO", "Failed to acquire SPI bus");
        return ret;
    }

    /* Send Burst Read Command (based on number of frames to read divided by word size of 32 bits) */
    uint32_t data = XENSIV_BGT60TR13C_SPI_BURST_MODE_CMD |
                        (XENSIV_BGT60TR13C_REG_FIFO_TR13C << XENSIV_BGT60TR13C_SPI_BURST_MODE_SADR_POS) | 
                        ((rx_buf_size / 32) << XENSIV_BGT60TR13C_SPI_BURST_MODE_LEN_POS);

    /* Setup tx data buffer */
    uint8_t tx_data[4] = {0};
    tx_data[0] = (data >> 24) & 0xFF;
    tx_data[1] = (data >> 16) & 0xFF;
    tx_data[2] = (data >> 8) & 0xFF;
    tx_data[3] = (data) & 0xFF;

    /* Create SPI transaction frame */
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * rx_buf_size,
        .tx_buffer = tx_data,
        .rxlength = 8 * rx_buf_size,
        .rx_buffer = frame_buf,
        .flags = 0
    };

    /* Perform the transaction */
    ret = spi_device_polling_transmit(spi, &t);

    /* De-aquire bus */
    spi_device_release_bus(spi);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed");
    }

    return ret;
}

esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data, bool verify_transaction) {
    /* Prepare the command byte (7-bit address + R/W bit) */
    uint8_t tx_buffer[4];

    tx_buffer[0] = (reg_addr << 1) | 0x01; // R/W bit set to 1 for write

    /* Prepare the data bytes (24-bit data) */
    tx_buffer[1] = (data >> 16) & 0xFF;
    tx_buffer[2] = (data >> 8) & 0xFF;
    tx_buffer[3] = data & 0xFF;

    /* Execute transaction */
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .flags = SPI_TRANS_USE_RXDATA
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi, &t));
    
    /* GSR0 error code logging */
    ESP_ERROR_CHECK_WITHOUT_ABORT(xensiv_bgt60tr13c_check_gsr0_err(t.rx_data[0]));

    if (verify_transaction) {
        uint32_t check_rx = xensiv_bgt60tr13c_get_reg(reg_addr) & 0x00FFFFFF;
        uint32_t check_tx = data & 0x00FFFFFF;
        if (check_rx != check_tx) {
            ESP_LOGW(TAG, "Verification for transaction failed. Tried to write: %lu, instead register reads: %lu", check_tx, check_rx);
        } else {
            ESP_LOGI(TAG, "Transaction verified. Successful write to register %lu", reg_addr);
        }
    }
    return ESP_OK;
}

uint32_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr) {
    /* Prepare the command byte (7-bit address + R/W bit) */
    uint8_t tx_buffer[4] = {0};
    tx_buffer[0] = (reg_addr << 1) | 0x00; // R/W bit set to 0 for read
    
    /* Execute transaction */
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .rxlength = 8 * sizeof(tx_buffer),
        .flags     = SPI_TRANS_USE_RXDATA
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi, &t));
    
    /* GSR0 error code logging */
    ESP_ERROR_CHECK_WITHOUT_ABORT(xensiv_bgt60tr13c_check_gsr0_err(t.rx_data[0]));

    return (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];
}

esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type) {
    uint32_t tmp;
    int32_t status;

    tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
    tmp |= (uint32_t)reset_type;
    status = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp, false);

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

esp_err_t get_frame_size(uint32_t *external_frame_size) {
    *external_frame_size = frame_size;
    return ESP_OK;
}

esp_err_t get_interrupt_frame_size_trigger(uint32_t *external_frame_size) {
    *external_frame_size = XENSIV_BGT60TR13C_IRQ_TRIGGER_FRAME_SIZE;
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_check_gsr0_err(uint8_t gsr0_err_code) {
    esp_err_t ret = ESP_OK;
    /* Assuming error code is 8 bits with MSB on the left */
    uint8_t error_masked = gsr0_err_code & (XENSIV_BGT60TR13C_REG_GSR0_FOU_ERR_MSK |
                                            XENSIV_BGT60TR13C_REG_GSR0_SPI_BURST_ERR_MSK |
                                            XENSIV_BGT60TR13C_REG_GSR0_CLK_NUM_ERR_MSK);

    if (error_masked & XENSIV_BGT60TR13C_REG_GSR0_CLK_NUM_ERR_MSK) {
        ESP_LOGE(TAG, "CLOCK NUMBER ERROR OCCURRED");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (error_masked & XENSIV_BGT60TR13C_REG_GSR0_SPI_BURST_ERR_MSK) {
        ESP_LOGE(TAG, "SPI BURST READ ERROR OCCURRED");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if ((error_masked & XENSIV_BGT60TR13C_REG_GSR0_FOU_ERR_MSK) && radar_configured) {
        ESP_LOGW(TAG, "RADAR FIFO OVERFLOW/UNDERFLOW ERROR DETECTED");
        ret = ESP_OK;
    }

    return ret;
}