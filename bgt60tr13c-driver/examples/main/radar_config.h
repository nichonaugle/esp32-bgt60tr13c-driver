#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Set to 1 to enable I2C test mode firmware, 0 for normal operation
#define I2C_TEST_MODE_ENABLED 0

// ===================================================================
// ===     EXACT Parameters from Python Script for Consistency     ===
// ===================================================================
#define PY_F_BANDWIDTH_HZ       (2000000000.0)
#define PY_TC_S                 (0.00005738)

// --- Sensor Hardware Parameters (from device header) ---
#define M_CHIRPS                (64)
#define N_SAMPLES_PER_CHIRP     (128)
#define NUM_RX_ANTENNAS         (1)
#define FS_HZ                   (2352941.0)

// --- Algorithm & Tuning Parameters ---
#define HIGH_RES_FFT_LEN        (1024)
#define N_RANGE_BINS            (HIGH_RES_FFT_LEN / 2)
#define C_MPS                   (299792458.0)

// ===================================================================
// ===         CORRECTED Range Calculation Formulas                ===
// ===================================================================
// R_max = (c * Fs * Tc) / (4 * B)
#define R_MAX_M                 ((C_MPS * FS_HZ * PY_TC_S) / (4.0 * PY_F_BANDWIDTH_HZ))

// delta_R = c / (2 * B)
#define RANGE_RESOLUTION_M      (C_MPS / (2.0 * PY_F_BANDWIDTH_HZ))


// --- GPIO and SPI Configuration ---
#define SPI_CS_PIN              GPIO_NUM_13
#define SPI_SCK_PIN             GPIO_NUM_27
#define SPI_MOSI_PIN            GPIO_NUM_25
#define SPI_MISO_PIN            GPIO_NUM_26
#define RADAR_IRQ_PIN           GPIO_NUM_4
#define I2C_MASTER_IRQ_PIN      GPIO_NUM_2

// --- Dynamic Configuration Structure ---
typedef struct {
    uint32_t frame_delay_ms;
    float background_alpha;
    uint32_t recalibration_interval_s;
    float near_range_bias_db;
    float far_range_bias_db;
    uint8_t presence_cfar_guards;
    uint8_t presence_cfar_refs;
    uint8_t history_len;
    uint8_t min_detections_in_history;
    float max_range_diff_m;
    bool motion_detected;
    bool presence_confirmed;
    bool enable_uart_plotting;
} radar_config_t;

// --- Function Prototypes ---
void radar_config_init(void);
radar_config_t* radar_config_get(void);
void radar_config_set_uart_plotting(bool enable);
void radar_config_set_frame_delay(uint32_t delay_ms);
void radar_config_set_presence_params(float bg_alpha, uint32_t recal_interval_s);
void radar_config_set_cfar_params(float near_bias, float far_bias, uint8_t guards, uint8_t refs);
void radar_config_set_temporal_filter_params(uint8_t hist_len, uint8_t min_dets, float max_range_diff);
void radar_config_set_motion_detected(bool detected);
void radar_config_set_presence_confirmed(bool confirmed);
bool radar_config_get_motion_detected_safe(void);
bool radar_config_get_presence_confirmed_safe(void);
esp_err_t radar_config_init_interrupt_pin(void);
void radar_config_trigger_motion_interrupt(void);

#endif /* RADAR_CONFIG_H */
