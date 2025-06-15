#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// --- Fixed Radar & System Parameters (from BGT60TR13C config) ---
#define M_CHIRPS 16
#define N_SAMPLES_PER_CHIRP 256
#define NUM_RX_ANTENNAS 3

// --- Calculated Radar Parameters ---
#define C_MPS 299792458.0
#define F_START_HZ 60e9
#define F_END_HZ 62e9
#define TC_S 0.0001935
#define B_HZ (F_END_HZ - F_START_HZ)
#define RANGE_RESOLUTION_M (C_MPS / (2.0 * B_HZ))

// Calculate max unambiguous range
#define FS_HZ (N_SAMPLES_PER_CHIRP / TC_S)
#define R_MAX_M ((FS_HZ / 2.0) * C_MPS * TC_S / (2.0 * B_HZ))

// --- FFT Configuration ---
#define RANGE_FFT_SIZE (N_SAMPLES_PER_CHIRP * 4)  // Zero-pad 4x
#define N_RANGE_BINS (RANGE_FFT_SIZE / 2)         // 512 bins

// --- GPIO and SPI Configuration ---
#define SPI_CS_PIN GPIO_NUM_13
#define SPI_SCK_PIN GPIO_NUM_27
#define SPI_MOSI_PIN GPIO_NUM_25
#define SPI_MISO_PIN GPIO_NUM_26
#define RADAR_IRQ_PIN GPIO_NUM_4

// --- Dynamic Configuration Structure ---
typedef struct {
    // Processing parameters
    uint8_t antenna_index;          // Which antenna to process (0-2)
    float useful_range_m;           // Range limit for processing
    
    // CFAR parameters
    uint8_t num_guard_cells;
    uint8_t num_ref_cells;
    float cfar_bias_db;
    
    // Debug settings
    bool enable_uart_plotting;
    
    // Frame capture settings
    uint32_t frame_delay_ms;        // Delay between frames
    
} radar_config_t;

// --- Configuration Management Functions ---
void radar_config_init(void);
radar_config_t* radar_config_get(void);
void radar_config_set_antenna_index(uint8_t index);
void radar_config_set_useful_range(float range_m);
void radar_config_set_cfar_params(uint8_t guard_cells, uint8_t ref_cells, float bias_db);
void radar_config_set_uart_plotting(bool enable);
void radar_config_set_frame_delay(uint32_t delay_ms);

#endif /* RADAR_CONFIG_H */