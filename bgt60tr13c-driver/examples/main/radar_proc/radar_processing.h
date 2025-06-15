#ifndef RADAR_PROCESSING_H
#define RADAR_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include "radar_config.h"
#include <esp_err.h>

// --- Processing Buffer Management ---
typedef struct {
    uint16_t *frame_buf_uint16;
    float *cpi_buf_float;
    float *fft_buffer;
    float *range_magnitudes_buf;
    float *range_profiles_db_buf;
    float *avg_range_profile_db;
    float *cfar_threshold_db;
    float *hamming_window;
    float *doppler_window;
} radar_buffers_t;

// --- Processing Functions ---
esp_err_t radar_processing_init(radar_buffers_t *buffers, uint32_t frame_size_samples);
void radar_processing_cleanup(radar_buffers_t *buffers);
void process_radar_frame(radar_buffers_t *buffers, uint32_t frame_count);

// --- Helper Functions ---
void deinterleave_antenna_data(radar_buffers_t *buffers, uint8_t antenna_index);
void apply_windowing(radar_buffers_t *buffers);
void perform_range_fft(radar_buffers_t *buffers);
void calculate_average_range_profile(radar_buffers_t *buffers, int useful_range_bins);
void perform_cfar_detection(radar_buffers_t *buffers, int useful_range_bins, 
                           uint8_t guard_cells, uint8_t ref_cells, float bias_db);
void detect_and_report_targets(radar_buffers_t *buffers, int useful_range_bins, 
                              uint32_t frame_count, bool enable_uart_plotting);

#endif /* RADAR_PROCESSING_H */