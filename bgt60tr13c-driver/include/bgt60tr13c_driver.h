#ifndef XENSIV_BGT60TR13C_H
#define XENSIV_BGT60TR13C_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "bgt60tr13c_regs.h"
#include "driver/gpio.h"

/* Reset types */
typedef enum {
    /*! Software reset.
        Resets all registers to default state.
        Resets all internal counters (e.g. shape, frame).
        Perform FIFO reset. Perform FSM reset */
    XENSIV_BGT60TR13C_RESET_SW = 0x1 << XENSIV_BGT60TR13C_REG_MAIN_RESET_POS,
    /*! FSM reset.
        Resets FSM to deep sleep mode.
        Resets FSM internal counters for channel/shape set and timers */
    XENSIV_BGT60TR13C_RESET_FSM = 0x2 << XENSIV_BGT60TR13C_REG_MAIN_RESET_POS,
    /*! FIFO reset.
        Reset the read and write pointers of the FIFO.
        Perform an implicit FSM reset */
    XENSIV_BGT60TR13C_RESET_FIFO = 0x4 << XENSIV_BGT60TR13C_REG_MAIN_RESET_POS
} xensiv_bgt60tr13c_reset_t;

/* Function prototypes for setup */
esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config);
esp_err_t xensiv_bgt60tr13c_configure();

/* Register control prototypes */
esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data, bool verify_transaction);
uint32_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr);
esp_err_t xensiv_bgt60tr13c_start_frame_capture();
esp_err_t xensiv_bgt60tr13c_fifo_read(uint8_t *frame_buf, uint32_t buf_size, uint32_t words_to_read);
esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type);
esp_err_t get_frame_size(uint32_t *external_frame_size);
esp_err_t get_interrupt_frame_size_trigger(uint32_t *external_frame_size);
esp_err_t xensiv_bgt60tr13c_check_gsr0_err(uint8_t gsr0_err_code);

// Additional utility functions
esp_err_t xensiv_bgt60tr13c_read_back_registers(void);
esp_err_t xensiv_bgt60tr13c_verify_configuration(void);
esp_err_t xensiv_bgt60tr13c_read_specific_registers(const uint32_t *reg_addresses, uint32_t num_regs);

#endif /* XENSIV_BGT60TR13C_H */
