#include <string.h>
#include <stdlib.h>
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

/* Determine size of received frame defined by [ samples * chirps * rx_antennas(3) ] */
static uint32_t frame_size = XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP * 
                                    XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME * 
                                    XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS;

/* Word reverse function for ESP32 - converts between big/little endian */
uint32_t xensiv_bgt60tr13c_platform_word_reverse(uint32_t word) {
    return ((word & 0x000000FF) << 24) |
           ((word & 0x0000FF00) << 8) |
           ((word & 0x00FF0000) >> 8) |
           ((word & 0xFF000000) >> 24);
}

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config) {
    /* Ensure the dev_config has been configured */
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
    
    /* Sets register to start frame capture. verification is false since it is a write only bit in the register */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tx_data, false), TAG, "Failed to start radar frame capture");

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_fifo_read(uint8_t *frame_buf, uint32_t buf_size, uint32_t words_to_read) {
    /* Ensure buffer size is valid - must be multiple of 3 for 24-bit FIFO words */
    if ((buf_size % 3) != 0) {
        ESP_LOGE(TAG, "Invalid buffer size. Must be a multiple of 3");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Acquire SPI bus */
    esp_err_t ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE("FIFO", "Failed to acquire SPI bus");
        return ret;
    }

    /* 
     * Build Burst Read Command 
     * According to datasheet Table 52: Use NBURSTS=0 for unbounded read
     */
    uint32_t burst_cmd = XENSIV_BGT60TR13C_SPI_BURST_MODE_CMD |
                         (XENSIV_BGT60TR13C_REG_FIFO_TR13C << XENSIV_BGT60TR13C_SPI_BURST_MODE_SADR_POS);
    // RWB=0 for read, NBURSTS=0 for unbounded - these are already 0 by default

    /* Apply word reversal for correct byte order on ESP32 */
    burst_cmd = xensiv_bgt60tr13c_platform_word_reverse(burst_cmd);

    /* 
     * Create buffers for the complete transaction:
     * TX: 4 bytes (burst command) + buf_size bytes (dummy data for clocking)
     * RX: 4 bytes (GSR0 + padding) + buf_size bytes (actual FIFO data)
     */
    uint32_t total_length = 4 + buf_size;
    uint8_t *tx_buffer = calloc(total_length, 1);
    uint8_t *rx_buffer = malloc(total_length);
    
    if (tx_buffer == NULL || rx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate transaction buffers");
        free(tx_buffer);
        free(rx_buffer);
        spi_device_release_bus(spi);
        return ESP_ERR_NO_MEM;
    }

    /* Copy burst command to TX buffer */
    memcpy(tx_buffer, &burst_cmd, 4);
    /* Rest of TX buffer is already zeros (dummy bytes for clocking out FIFO data) */

    /* 
     * Single SPI transaction: Send command + dummy bytes, receive GSR0 + FIFO data
     * This follows the datasheet Figure 54 protocol in one continuous transaction
     */
    spi_transaction_t transaction = {
        .cmd = 0,
        .addr = 0,
        .length = total_length * 8,      // Total TX length in bits
        .tx_buffer = tx_buffer,
        .rxlength = total_length * 8,    // Total RX length in bits  
        .rx_buffer = rx_buffer,
        .flags = 0
    };

    ret = spi_device_polling_transmit(spi, &transaction);
    
    if (ret == ESP_OK) {
        /* Check GSR0 for errors (first byte of received data) */
        uint8_t gsr0_status = rx_buffer[0];
        ret = xensiv_bgt60tr13c_check_gsr0_err(gsr0_status);
        
        if (ret == ESP_OK) {
            /* Copy FIFO data (skip first 4 bytes which are GSR0 + padding) */
            memcpy(frame_buf, &rx_buffer[4], buf_size);
        } else {
            ESP_LOGE(TAG, "GSR0 error detected: 0x%02X", gsr0_status);
        }
    } else {
        ESP_LOGE(TAG, "SPI FIFO transaction failed");
    }

    /* Clean up buffers */
    free(tx_buffer);
    free(rx_buffer);

    /* Release bus */
    spi_device_release_bus(spi);

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

    // Skip GSR0 byte (t.rx_data[0]) and only return the 24-bit register data
    return (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];
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

esp_err_t xensiv_bgt60tr13c_read_back_registers(void) {
    ESP_LOGI(TAG, "=== Reading Back All Configured Registers ===");
    
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) {
        uint32_t expected_val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((expected_val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t expected_data = ((expected_val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                    XENSIV_BGT60TR13C_SPI_DATA_POS);
        
        uint32_t actual_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        
        ESP_LOGI(TAG, "Reg[0x%02lX]: Expected=0x%06lX, Actual=0x%06lX %s", 
                 reg_addr, expected_data, actual_data, 
                 (expected_data == actual_data) ? "✓" : "✗");
        
        // Small delay to prevent overwhelming the log
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "=== Register Readback Complete ===");
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_verify_configuration(void) {
    ESP_LOGI(TAG, "=== Verifying Configuration ===");
    
    uint32_t mismatched_registers = 0;
    
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) {
        uint32_t expected_val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((expected_val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t expected_data = ((expected_val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                    XENSIV_BGT60TR13C_SPI_DATA_POS);
        
        uint32_t actual_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        
        if (expected_data != actual_data) {
            ESP_LOGE(TAG, "MISMATCH Reg[0x%02lX]: Expected=0x%06lX, Actual=0x%06lX", 
                     reg_addr, expected_data, actual_data);
            mismatched_registers++;
        }
    }
    
    if (mismatched_registers == 0) {
        ESP_LOGI(TAG, "All %d registers match expected values", XENSIV_BGT60TR13C_CONF_NUM_REGS);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "%lu registers do not match expected values", mismatched_registers);
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t xensiv_bgt60tr13c_read_specific_registers(const uint32_t *reg_addresses, uint32_t num_regs) {
    ESP_LOGI(TAG, "=== Reading Specific Registers ===");
    
    for(uint32_t i = 0; i < num_regs; i++) {
        uint32_t reg_addr = reg_addresses[i];
        uint32_t reg_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        ESP_LOGI(TAG, "Reg[0x%02lX] = 0x%06lX", reg_addr, reg_data);
    }
    
    return ESP_OK;
}