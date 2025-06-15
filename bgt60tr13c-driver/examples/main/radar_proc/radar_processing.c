#include "radar_processing.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "RADAR_PROCESSING";

esp_err_t radar_processing_init(radar_buffers_t *buffers, uint32_t frame_size_samples) {
    // Allocate all processing buffers
    buffers->frame_buf_uint16 = (uint16_t *)malloc(frame_size_samples * sizeof(uint16_t));
    buffers->cpi_buf_float = (float *)malloc(M_CHIRPS * N_SAMPLES_PER_CHIRP * sizeof(float));
    buffers->fft_buffer = (float *)malloc(RANGE_FFT_SIZE * 2 * sizeof(float));
    buffers->range_magnitudes_buf = (float *)malloc(M_CHIRPS * N_RANGE_BINS * sizeof(float));
    buffers->range_profiles_db_buf = (float *)malloc(M_CHIRPS * N_RANGE_BINS * sizeof(float));
    buffers->avg_range_profile_db = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->cfar_threshold_db = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->hamming_window = (float *)malloc(N_SAMPLES_PER_CHIRP * sizeof(float));
    buffers->doppler_window = (float *)malloc(M_CHIRPS * sizeof(float));
    
    if (!buffers->frame_buf_uint16 || !buffers->cpi_buf_float || !buffers->fft_buffer || 
        !buffers->avg_range_profile_db || !buffers->cfar_threshold_db || 
        !buffers->hamming_window || !buffers->doppler_window || 
        !buffers->range_magnitudes_buf || !buffers->range_profiles_db_buf) {
        ESP_LOGE(TAG, "Failed to allocate processing buffers!");
        radar_processing_cleanup(buffers);
        return ESP_ERR_NO_MEM;
    }

    // Initialize DSP
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    
    // Check if FFT size is supported
    if (RANGE_FFT_SIZE > CONFIG_DSP_MAX_FFT_SIZE) {
        ESP_LOGE(TAG, "FFT size %d exceeds CONFIG_DSP_MAX_FFT_SIZE %d", RANGE_FFT_SIZE, CONFIG_DSP_MAX_FFT_SIZE);
        ESP_LOGE(TAG, "Increase CONFIG_DSP_MAX_FFT_SIZE in menuconfig to at least %d", RANGE_FFT_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Generate Hamming windows to match Python exactly
    // Range window: signal.windows.hamming(N_SAMPLES_PER_CHIRP)
    for (int i = 0; i < N_SAMPLES_PER_CHIRP; i++) {
        buffers->hamming_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (N_SAMPLES_PER_CHIRP - 1));
    }
    
    // Doppler window: signal.windows.hamming(M_CHIRPS) 
    for (int i = 0; i < M_CHIRPS; i++) {
        buffers->doppler_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (M_CHIRPS - 1));
    }

    return ESP_OK;
}

void radar_processing_cleanup(radar_buffers_t *buffers) {
    if (buffers->frame_buf_uint16) { free(buffers->frame_buf_uint16); buffers->frame_buf_uint16 = NULL; }
    if (buffers->cpi_buf_float) { free(buffers->cpi_buf_float); buffers->cpi_buf_float = NULL; }
    if (buffers->fft_buffer) { free(buffers->fft_buffer); buffers->fft_buffer = NULL; }
    if (buffers->range_magnitudes_buf) { free(buffers->range_magnitudes_buf); buffers->range_magnitudes_buf = NULL; }
    if (buffers->range_profiles_db_buf) { free(buffers->range_profiles_db_buf); buffers->range_profiles_db_buf = NULL; }
    if (buffers->avg_range_profile_db) { free(buffers->avg_range_profile_db); buffers->avg_range_profile_db = NULL; }
    if (buffers->cfar_threshold_db) { free(buffers->cfar_threshold_db); buffers->cfar_threshold_db = NULL; }
    if (buffers->hamming_window) { free(buffers->hamming_window); buffers->hamming_window = NULL; }
    if (buffers->doppler_window) { free(buffers->doppler_window); buffers->doppler_window = NULL; }
}

void deinterleave_antenna_data(radar_buffers_t *buffers, uint8_t antenna_index) {
    float (*cpi_frame)[N_SAMPLES_PER_CHIRP] = (float (*)[N_SAMPLES_PER_CHIRP])buffers->cpi_buf_float;
    
    for (int chirp = 0; chirp < M_CHIRPS; ++chirp) {
        for (int sample = 0; sample < N_SAMPLES_PER_CHIRP; ++sample) {
            int flat_idx = (chirp * N_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS) + (sample * NUM_RX_ANTENNAS) + antenna_index;
            cpi_frame[chirp][sample] = (float)buffers->frame_buf_uint16[flat_idx];
        }
    }
}

void apply_windowing(radar_buffers_t *buffers) {
    float (*cpi_frame)[N_SAMPLES_PER_CHIRP] = (float (*)[N_SAMPLES_PER_CHIRP])buffers->cpi_buf_float;
    
    for (int i = 0; i < M_CHIRPS; i++) {
        for (int j = 0; j < N_SAMPLES_PER_CHIRP; j++) {
            // Apply both windows like Python: 
            // cpi_data_windowed_both = cpi_data_windowed_range * doppler_window[:, np.newaxis]
            cpi_frame[i][j] = cpi_frame[i][j] * buffers->hamming_window[j] * buffers->doppler_window[i];
        }
    }
}

void perform_range_fft(radar_buffers_t *buffers) {
    float (*cpi_frame)[N_SAMPLES_PER_CHIRP] = (float (*)[N_SAMPLES_PER_CHIRP])buffers->cpi_buf_float;
    float (*range_magnitudes)[N_RANGE_BINS] = (float (*)[N_RANGE_BINS])buffers->range_magnitudes_buf;
    float (*range_profiles_db)[N_RANGE_BINS] = (float (*)[N_RANGE_BINS])buffers->range_profiles_db_buf;
    
    for (int i = 0; i < M_CHIRPS; i++) {
        // Clear the entire FFT buffer first (for zero-padding)
        memset(buffers->fft_buffer, 0, RANGE_FFT_SIZE * 2 * sizeof(float));
        
        // Fill with windowed data (already windowed)
        for (int j = 0; j < N_SAMPLES_PER_CHIRP; j++) {
            buffers->fft_buffer[2 * j] = cpi_frame[i][j];
            buffers->fft_buffer[2 * j + 1] = 0;
        }
        // Remaining samples stay zero (zero-padding)
        
        // Perform FFT on the zero-padded buffer
        dsps_fft2r_fc32(buffers->fft_buffer, RANGE_FFT_SIZE);
        dsps_bit_rev_fc32(buffers->fft_buffer, RANGE_FFT_SIZE);
        
        // Calculate magnitude and convert to dB (with potential scaling fix)
        for (int k = 0; k < N_RANGE_BINS; k++) {
            float magnitude = sqrtf(buffers->fft_buffer[2 * k] * buffers->fft_buffer[2 * k] + 
                                   buffers->fft_buffer[2 * k + 1] * buffers->fft_buffer[2 * k + 1]);
            
            // Apply scaling to match NumPy FFT (ESP-DSP doesn't scale by default)
            magnitude = magnitude / RANGE_FFT_SIZE;
            
            range_magnitudes[i][k] = magnitude;
            range_profiles_db[i][k] = 20.0f * log10f(magnitude + 1e-9f);
        }
    }
}

void calculate_average_range_profile(radar_buffers_t *buffers, int useful_range_bins) {
    float (*range_profiles_db)[N_RANGE_BINS] = (float (*)[N_RANGE_BINS])buffers->range_profiles_db_buf;
    
    // Average in dB domain (exactly like Python) - only for useful range
    for (int i = 0; i < useful_range_bins; i++) {
        buffers->avg_range_profile_db[i] = 0;
        for (int j = 0; j < M_CHIRPS; j++) { 
            buffers->avg_range_profile_db[i] += range_profiles_db[j][i]; 
        }
        buffers->avg_range_profile_db[i] /= M_CHIRPS;
    }
}

void perform_cfar_detection(radar_buffers_t *buffers, int useful_range_bins, 
                           uint8_t guard_cells, uint8_t ref_cells, float bias_db) {
    int pad = guard_cells + ref_cells;
    
    // Initialize thresholds (like Python's np.pad with np.nan)
    for (int i = 0; i < useful_range_bins; i++) {
        buffers->cfar_threshold_db[i] = 1000.0f; // High value like np.nan
    }

    // Calculate CFAR threshold (matching Python's sliding window approach)
    for (int i = pad; i < useful_range_bins - pad; i++) {
        float noise_sum = 0;
        int ref_count = 0;
        
        // Left reference cells
        for (int j = i - pad; j < i - guard_cells; j++) { 
            noise_sum += buffers->avg_range_profile_db[j]; 
            ref_count++;
        }
        // Right reference cells  
        for (int j = i + guard_cells + 1; j <= i + pad; j++) { 
            noise_sum += buffers->avg_range_profile_db[j]; 
            ref_count++;
        }
        
        // Average + bias
        buffers->cfar_threshold_db[i] = (noise_sum / ref_count) + bias_db;
    }
}

void detect_and_report_targets(radar_buffers_t *buffers, int useful_range_bins, 
                              uint32_t frame_count, bool enable_uart_plotting) {
    // With 4x zero-padding, each bin is 1/4 the original range resolution
    float effective_range_resolution = RANGE_RESOLUTION_M / 4.0f;
    
    // Detect targets
    bool target_found = false;
    printf("--- Target Detection Results (Frame %lu) ---\n", frame_count);
    
    for (int i = 0; i < useful_range_bins; i++) {
        if (buffers->avg_range_profile_db[i] > buffers->cfar_threshold_db[i]) {
            float range = (float)i * effective_range_resolution;
            float db_magnitude = buffers->avg_range_profile_db[i];
            
            printf("Target detected at range: %.3f m (Magnitude: %.2f dB, Threshold: %.2f dB)\n", 
                   range, db_magnitude, buffers->cfar_threshold_db[i]);
            target_found = true;
        }
    }
    
    if (!target_found) {
        printf("No targets detected.\n");
    }
    
    // Simple debug info
    float max_signal_db = -1000.0f;
    int max_signal_bin = -1;
    
    for (int i = 0; i < useful_range_bins; i++) {
        if (buffers->avg_range_profile_db[i] > max_signal_db) {
            max_signal_db = buffers->avg_range_profile_db[i];
            max_signal_bin = i;
        }
    }
    
    printf("Max signal: %.2f dB at %.3f m (bin %d)\n", 
           max_signal_db, max_signal_bin * effective_range_resolution, max_signal_bin);
    
    if (enable_uart_plotting) {
        // Output range profile and CFAR data for Python plotting (only useful range)
        printf("RANGE_PROFILE_START\n");
        printf("RANGE_DATA:");
        for (int i = 0; i < useful_range_bins; i++) {
            printf("%.2f", buffers->avg_range_profile_db[i]);
            if (i < useful_range_bins - 1) printf(",");
        }
        printf("\n");
        
        printf("CFAR_DATA:");
        for (int i = 0; i < useful_range_bins; i++) {
            if (buffers->cfar_threshold_db[i] > 500.0f) {
                printf("nan");  // For edge bins with high threshold
            } else {
                printf("%.2f", buffers->cfar_threshold_db[i]);
            }
            if (i < useful_range_bins - 1) printf(",");
        }
        printf("\n");
        
        printf("RANGE_BINS:");
        for (int i = 0; i < useful_range_bins; i++) {
            printf("%.4f", i * effective_range_resolution);
            if (i < useful_range_bins - 1) printf(",");
        }
        printf("\n");
        printf("RANGE_PROFILE_END\n");
    }
    
    printf("----------------------------------------\n\n");
}

void process_radar_frame(radar_buffers_t *buffers, uint32_t frame_count) {
    radar_config_t *config = radar_config_get();
    
    // With 4x zero-padding, each bin is 1/4 the original range resolution
    float effective_range_resolution = RANGE_RESOLUTION_M / 4.0f;
    
    // Calculate useful range bins
    int useful_range_bins = (int)(config->useful_range_m / effective_range_resolution);
    if (useful_range_bins > N_RANGE_BINS) useful_range_bins = N_RANGE_BINS;

    // 1. De-interleave and convert data for the selected antenna
    deinterleave_antenna_data(buffers, config->antenna_index);

    // 2. Apply both range and doppler windowing
    apply_windowing(buffers);
    
    // 3. Perform Range-FFT with zero-padding
    perform_range_fft(buffers);
    
    // 4. Average in dB domain - only for useful range
    calculate_average_range_profile(buffers, useful_range_bins);

    // 5. Perform CFAR  - only for useful range
    perform_cfar_detection(buffers, useful_range_bins, 
                          config->num_guard_cells, config->num_ref_cells, config->cfar_bias_db);

    // 6. Detect and report targets
    detect_and_report_targets(buffers, useful_range_bins, frame_count, config->enable_uart_plotting);
}