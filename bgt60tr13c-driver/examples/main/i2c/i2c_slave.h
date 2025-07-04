#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// I2C Slave Configuration
#define I2C_SLAVE_PORT          I2C_NUM_0
#define I2C_SLAVE_ADDR          0x28
#define I2C_SLAVE_SDA_PIN       GPIO_NUM_21
#define I2C_SLAVE_SCL_PIN       GPIO_NUM_22
#define I2C_SLAVE_RX_BUF_LEN    256
#define I2C_SLAVE_TX_BUF_LEN    256

// Register Map for I2C Communication
typedef enum {
    // --- State Registers (Read-Only) ---
    REG_TEST_MODE_STATUS        = 0x00,
    REG_MOTION_DETECTED         = 0x01, // Raw, per-frame detection
    REG_PRESENCE_CONFIRMED      = 0x02, // Temporally filtered detection
    REG_OCCUPANCY_STATE         = 0x03, // Time-delayed occupancy
    REG_SYSTEM_STATUS           = 0x0F,

    // --- Control & Tuning Registers (Read/Write) ---
    // General
    REG_FRAME_DELAY_MS          = 0x10,
    REG_OCCUPANCY_DELAY_S       = 0x11,
    
    // Processing
    REG_COHERENT_INT_FACTOR     = 0x18,
    REG_MOVING_AVG_WINDOW       = 0x19,
    REG_BACKGROUND_ALPHA        = 0x20,
    REG_RECALIBRATE_NOW         = 0x21,
    
    // CFAR
    REG_CFAR_NEAR_BIAS_DB       = 0x30,
    REG_CFAR_FAR_BIAS_DB        = 0x31,
    REG_CFAR_GUARDS             = 0x32,
    REG_CFAR_REFS               = 0x33,

    // Temporal Filter
    REG_HISTORY_LEN             = 0x40,
    REG_MIN_DETECTIONS          = 0x41,
    REG_MAX_RANGE_DIFF_M        = 0x42,

    // Misc
    REG_UART_PLOTTING           = 0x50,

} i2c_register_t;


// I2C Slave Functions
esp_err_t i2c_slave_init(void);
void i2c_slave_task(void *pvParameters);
void i2c_slave_task_create(void);

#endif /* I2C_SLAVE_H */
