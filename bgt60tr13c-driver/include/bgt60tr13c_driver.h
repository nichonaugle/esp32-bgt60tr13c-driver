#ifndef XENSIV_BGT60TR13C_H
#define XENSIV_BGT60TR13C_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* Reset types */
typedef enum {
    XENSIV_BGT60TR13C_RESET_SW = 1,
    XENSIV_BGT60TR13C_RESET_FSM = 2
} xensiv_bgt60tr13c_reset_t;

/* Function prototypes */
esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config, gpio_num_t interrupt_pin);
void xensiv_bgt60tr13c_radar_task(void *pvParameters);
void IRAM_ATTR gpio_radar_isr_handler(void *arg);

/* Register control prototypes */
esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data);
uint32_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr);
esp_err_t xensiv_bgt60tr13c_fifo_read(uint32_t *frame_buf, uint32_t rx_buf_size);
esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type);

#endif /* XENSIV_BGT60TR13C_H */
