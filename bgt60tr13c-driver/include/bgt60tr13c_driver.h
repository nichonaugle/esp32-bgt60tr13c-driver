#ifndef XENSIV_BGT60TR13C_H
#define XENSIV_BGT60TR13C_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* Reset types */
typedef enum {
    XENSIV_BGT60TR13C_RESET_SW = 1,
    XENSIV_BGT60TR13C_RESET_FSM = 2
} xensiv_bgt60tr13c_reset_t;

typedef struct {
    spi_host_device_t spi_host;
    spi_device_interface_config_t dev_config;
    spi_device_handle_t spi_handle;
} esp_spi_config_t;

/* Function prototypes */
esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config);
esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data_to_send);
esp_err_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr, uint32_t *data_to_recieve);
esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type);

//uint16_t xensiv_bgt60tr13c_get_fifo_size(const xensiv_bgt60tr13c_t* dev);
//esp_err_t xensiv_bgt60tr13c_get_fifo_data(const xensiv_bgt60tr13c_t* dev, uint16_t* data, uint32_t num_samples);
//esp_err_t xensiv_bgt60tr13c_start_frame(const xensiv_bgt60tr13c_t* dev, bool start);



#endif /* XENSIV_BGT60TR13C_H */
