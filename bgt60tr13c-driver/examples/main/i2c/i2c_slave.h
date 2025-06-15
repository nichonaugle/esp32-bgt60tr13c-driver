#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

// --- I2C Slave Configuration ---
#define I2C_SLAVE_PORT          I2C_NUM_0
#define I2C_SLAVE_ADDR          0x28          // 7-bit slave address
#define I2C_SLAVE_SDA_PIN       GPIO_NUM_21   // Default SDA pin
#define I2C_SLAVE_SCL_PIN       GPIO_NUM_22   // Default SCL pin
#define I2C_SLAVE_RX_BUF_LEN    128           // Receive buffer length
#define I2C_SLAVE_TX_BUF_LEN    128           // Transmit buffer length

// --- Register Map for I2C Communication ---
typedef enum {
    REG_ANTENNA_INDEX = 0x00,     // Select antenna (0-2)
    REG_USEFUL_RANGE = 0x01,      // Useful range in meters (float)
    REG_GUARD_CELLS = 0x05,       // CFAR guard cells
    REG_REF_CELLS = 0x06,         // CFAR reference cells  
    REG_CFAR_BIAS = 0x07,         // CFAR bias in dB (float)
    REG_UART_PLOTTING = 0x0B,     // Enable/disable UART plotting
    REG_FRAME_DELAY = 0x0C,       // Frame delay in ms
    REG_SYSTEM_STATUS = 0x10,     // System status (read-only)
    REG_FRAME_COUNT = 0x11,       // Current frame count (read-only)
} i2c_register_t;

// --- I2C Slave Functions (to be implemented) ---
esp_err_t i2c_slave_init(void);
esp_err_t i2c_slave_deinit(void);
void i2c_slave_task(void *pvParameters);

// --- Register Access Functions (to be implemented) ---
esp_err_t i2c_slave_read_register(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_slave_write_register(uint8_t reg_addr, const uint8_t *data, size_t len);

// --- Task Management ---
void i2c_slave_task_create(void);

#endif /* I2C_SLAVE_H */