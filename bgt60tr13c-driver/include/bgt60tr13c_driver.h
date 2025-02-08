#ifndef XENSIV_BGT60TR13C_H
#define XENSIV_BGT60TR13C_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Error codes for the driver */
#define XENSIV_BGT60TR13C_STATUS_OK                   0
#define XENSIV_BGT60TR13C_STATUS_DEV_ERROR            -1
#define XENSIV_BGT60TR13C_STATUS_TIMEOUT_ERROR        -2
#define XENSIV_BGT60TR13C_STATUS_GSR0_ERROR           -3

/* Reset types */
typedef enum {
    XENSIV_BGT60TR13C_RESET_SW = 1,
    XENSIV_BGT60TR13C_RESET_FSM = 2
} xensiv_bgt60tr13c_reset_t;

typedef struct {
    spi_host_device_t spi_host = NULL, 
    spi_device_interface_config_t dev_config = NULL, 
    spi_device_handle_t spi_handle = NULL
} esp_spi_config_t;

/* Function prototypes */
esp_err_t xensiv_bgt60tr13c_init(spi_device_interface_config_t dev_config, spi_device_handle_t spi_handle);
esp_err_t xensiv_bgt60tr13c_config(xensiv_bgt60tr13c_t* dev, const uint32_t* regs, uint32_t len);
esp_err_t xensiv_bgt60tr13c_set_reg(const xensiv_bgt60tr13c_t* dev, uint32_t reg_addr, uint32_t data);
esp_err_t xensiv_bgt60tr13c_get_reg(const xensiv_bgt60tr13c_t* dev, uint32_t reg_addr, uint32_t* data);
esp_err_t xensiv_bgt60tr13c_soft_reset(const xensiv_bgt60tr13c_t* dev, xensiv_bgt60tr13c_reset_t reset_type);
uint16_t xensiv_bgt60tr13c_get_fifo_size(const xensiv_bgt60tr13c_t* dev);
esp_err_t xensiv_bgt60tr13c_get_fifo_data(const xensiv_bgt60tr13c_t* dev, uint16_t* data, uint32_t num_samples);
esp_err_t xensiv_bgt60tr13c_start_frame(const xensiv_bgt60tr13c_t* dev, bool start);

#endif /* XENSIV_BGT60TR13C_H */
