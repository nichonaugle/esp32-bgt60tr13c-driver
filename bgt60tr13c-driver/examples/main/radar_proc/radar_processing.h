#ifndef RADAR_PROCESSING_H
#define RADAR_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include "radar_config.h"
#include "esp_err.h"

// ===================================================================
// ===            NEW: Processing Buffer Management              ===
// ===================================================================
typedef struct {
    // --- Raw Data & Staging ---
    uint16_t *raw_frame_buf;         // Holds the full, interleaved frame from the sensor
    float *stationary_data;          // Holds a single, de-interleaved, background-subtracted chirp
    float *fft_input_output;         // Buffer for in-place FFT

    // --- Background Model ---
    float *background_model;         // Adaptive background model (M_CHIRPS x N_SAMPLES_PER_CHIRP)

    // --- Range Profile ---
    float *profile_accumulator;      // Accumulates FFT magnitudes across chirps
    float *mean_profile_db;          // The final, averaged range profile in dB
    float *cfar_threshold_db;        // The calculated CFAR threshold

    // --- Windows & Biases ---
    float *hanning_window;           // Hanning window for range FFT
    float *presence_cfar_bias_array; // Linearly interpolated bias for CFAR

    // --- Presence History (for temporal filtering) ---
    // A circular buffer holding the ranges of detected targets for recent frames.
    // Dimensions: [history_len][max_detections_per_frame]
    float **presence_history;
    uint8_t *history_detection_counts; // Number of detections in each history frame
    int history_write_idx;             // Current position in the circular buffer
    
} radar_buffers_t;


// ===================================================================
// ===               NEW: Processing Core Functions                ===
// ===================================================================

/**
 * @brief Allocates and initializes all required processing buffers and states.
 */
esp_err_t radar_processing_init(radar_buffers_t *buffers, radar_config_t* config, uint32_t frame_size_samples);

/**
 * @brief Frees all allocated processing buffers.
 */
void radar_processing_cleanup(radar_buffers_t *buffers, radar_config_t* config);

/**
 * @brief Processes a complete radar frame to detect presence.
 */
void process_radar_frame(radar_buffers_t *buffers, radar_config_t* config, uint32_t frame_count);

/**
 * @brief Flags the processing task to reset the background model on the next frame.
 * Safe to call from other tasks (like I2C).
 */
void radar_processing_recalibrate_now(void);

/**
 * @brief Gets the current confirmed presence state.
 * Safe to call from other tasks.
 */
bool radar_processing_get_presence_state(void);


#endif /* RADAR_PROCESSING_H */
