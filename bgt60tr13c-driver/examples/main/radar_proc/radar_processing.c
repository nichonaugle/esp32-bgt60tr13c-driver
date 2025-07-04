// radar_processing.c

#include "radar_processing.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_DETECTIONS_PER_FRAME 20
static const char *TAG = "RADAR_PROCESSING";

static bool g_background_initialized = false;
static time_t g_last_recalibration_time = 0;
static volatile bool g_force_recalibration_flag = false;

static inline time_t get_time_s(void) {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return spec.tv_sec;
}

static bool is_detection_persistent(radar_buffers_t* buffers, radar_config_t* config) {
    int frames_with_detections = 0;
    for (int i = 0; i < config->history_len; i++) {
        if (buffers->history_detection_counts[i] > 0) {
            frames_with_detections++;
        }
    }

    if (frames_with_detections < config->min_detections_in_history) {
        return false;
    }

    float* latest_detections = buffers->presence_history[buffers->history_write_idx];
    uint8_t latest_count = buffers->history_detection_counts[buffers->history_write_idx];

    for (int i = 0; i < latest_count; i++) {
        float latest_det_range = latest_detections[i];
        int matches = 0;

        for (int j = 0; j < config->history_len; j++) {
            if (j == buffers->history_write_idx) continue;

            float* past_detections = buffers->presence_history[j];
            uint8_t past_count = buffers->history_detection_counts[j];

            for (int k = 0; k < past_count; k++) {
                if (fabsf(latest_det_range - past_detections[k]) <= config->max_range_diff_m) {
                    matches++;
                    break;
                }
            }
        }
        if ((matches + 1) >= config->min_detections_in_history) {
            return true;
        }
    }
    return false;
}

esp_err_t radar_processing_init(radar_buffers_t *buffers, radar_config_t* config, uint32_t frame_size_samples) {
    buffers->raw_frame_buf = (uint16_t *)malloc(frame_size_samples * sizeof(uint16_t));
    buffers->stationary_data = (float *)malloc(N_SAMPLES_PER_CHIRP * sizeof(float));
    buffers->fft_input_output = (float *)malloc(HIGH_RES_FFT_LEN * 2 * sizeof(float));
    buffers->integrated_fft_iq = (float *)calloc(HIGH_RES_FFT_LEN * 2, sizeof(float));
    buffers->background_model = (float *)calloc(M_CHIRPS * N_SAMPLES_PER_CHIRP, sizeof(float));
    buffers->profile_accumulator = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->mean_profile_db = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->cfar_threshold_db = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->hanning_window = (float *)malloc(N_SAMPLES_PER_CHIRP * sizeof(float));
    buffers->presence_cfar_bias_array = (float *)malloc(N_RANGE_BINS * sizeof(float));
    buffers->presence_history = (float **)malloc(config->history_len * sizeof(float *));
    for (int i = 0; i < config->history_len; i++) {
        buffers->presence_history[i] = (float *)calloc(MAX_DETECTIONS_PER_FRAME, sizeof(float));
    }
    buffers->history_detection_counts = (uint8_t *)calloc(config->history_len, sizeof(uint8_t));
    buffers->history_write_idx = 0;
    if (!buffers->raw_frame_buf || !buffers->stationary_data || !buffers->fft_input_output || !buffers->integrated_fft_iq || !buffers->background_model || !buffers->profile_accumulator || !buffers->mean_profile_db || !buffers->cfar_threshold_db || !buffers->hanning_window || !buffers->presence_cfar_bias_array || !buffers->presence_history || !buffers->history_detection_counts) {
        ESP_LOGE(TAG, "Failed to allocate one or more processing buffers!");
        radar_processing_cleanup(buffers, config);
        return ESP_ERR_NO_MEM;
    }
    dsps_fft2r_init_fc32(NULL, HIGH_RES_FFT_LEN);
    dsps_wind_hann_f32(buffers->hanning_window, N_SAMPLES_PER_CHIRP);

    // The CFAR bias interpolates linearly across the entire range.
    ESP_LOGI(TAG, "Initializing CFAR bias with linear interpolation.");
    for (int i = 0; i < N_RANGE_BINS; i++) {
        float ratio = (float)i / (float)(N_RANGE_BINS - 1);
        buffers->presence_cfar_bias_array[i] = config->near_range_bias_db + (config->far_range_bias_db - config->near_range_bias_db) * ratio;
    }
    
    g_last_recalibration_time = get_time_s();
    return ESP_OK;
}

void radar_processing_cleanup(radar_buffers_t *buffers, radar_config_t* config) {
    if(buffers == NULL) return;
    free(buffers->raw_frame_buf); free(buffers->stationary_data); free(buffers->fft_input_output); free(buffers->integrated_fft_iq); free(buffers->background_model); free(buffers->profile_accumulator); free(buffers->mean_profile_db); free(buffers->cfar_threshold_db); free(buffers->hanning_window); free(buffers->presence_cfar_bias_array);
    if (buffers->presence_history) { if(config != NULL) { for (int i = 0; i < config->history_len; i++) { free(buffers->presence_history[i]); } } free(buffers->presence_history); }
    free(buffers->history_detection_counts);
}

void radar_processing_recalibrate_now(void) { g_force_recalibration_flag = true; }

void process_radar_frame(radar_buffers_t *buffers, radar_config_t* config, uint32_t frame_count) {
    if (g_force_recalibration_flag || (get_time_s() - g_last_recalibration_time > config->recalibration_interval_s)) {
        ESP_LOGI(TAG, "Recalibration triggered. Resetting background model.");
        g_background_initialized = false;
        g_last_recalibration_time = get_time_s();
        g_force_recalibration_flag = false;
    }

    if (!g_background_initialized) {
        for (int c = 0; c < M_CHIRPS; c++) {
            for (int s = 0; s < N_SAMPLES_PER_CHIRP; s++) {
                buffers->background_model[c * N_SAMPLES_PER_CHIRP + s] = (float)buffers->raw_frame_buf[(c * N_SAMPLES_PER_CHIRP + s) * NUM_RX_ANTENNAS];
            }
        }
        g_background_initialized = true;
        ESP_LOGI(TAG, "Background model initialized.");
        return;
    }

    memset(buffers->profile_accumulator, 0, N_RANGE_BINS * sizeof(float));

    // Use coherent integration factor from config
    uint8_t int_factor = config->coherent_integration_factor;
    if (int_factor == 0 || M_CHIRPS % int_factor != 0) {
        int_factor = 1; // Fallback to 1 if config is invalid
    }
    const int num_integrated_chirps = M_CHIRPS / int_factor;

    for (int i = 0; i < num_integrated_chirps; i++) {
        memset(buffers->integrated_fft_iq, 0, HIGH_RES_FFT_LEN * 2 * sizeof(float));

        for (int j = 0; j < int_factor; j++) {
            int c = i * int_factor + j;

            for (int s = 0; s < N_SAMPLES_PER_CHIRP; s++) {
                float raw_sample = (float)buffers->raw_frame_buf[(c * N_SAMPLES_PER_CHIRP + s) * NUM_RX_ANTENNAS];
                float* bg_sample = &buffers->background_model[c * N_SAMPLES_PER_CHIRP + s];
                buffers->stationary_data[s] = raw_sample - *bg_sample;
                *bg_sample = (1.0f - config->background_alpha) * (*bg_sample) + config->background_alpha * raw_sample;
            }
            dsps_mul_f32_ae32(buffers->stationary_data, buffers->hanning_window, buffers->stationary_data, N_SAMPLES_PER_CHIRP, 1, 1, 1);

            for (int s = 0; s < N_SAMPLES_PER_CHIRP; s++) {
                buffers->fft_input_output[2 * s] = buffers->stationary_data[s];
                buffers->fft_input_output[2 * s + 1] = 0;
            }
            memset(&buffers->fft_input_output[2 * N_SAMPLES_PER_CHIRP], 0, (HIGH_RES_FFT_LEN - N_SAMPLES_PER_CHIRP) * 2 * sizeof(float));
            
            dsps_fft2r_fc32(buffers->fft_input_output, HIGH_RES_FFT_LEN);
            dsps_bit_rev_fc32(buffers->fft_input_output, HIGH_RES_FFT_LEN);
            
            for (int k = 0; k < HIGH_RES_FFT_LEN * 2; k++) {
                buffers->integrated_fft_iq[k] += buffers->fft_input_output[k];
            }
        }

        for (int k = 0; k < N_RANGE_BINS; k++) {
            float I = buffers->integrated_fft_iq[2*k];
            float Q = buffers->integrated_fft_iq[2*k+1];
            float mag = sqrtf(I * I + Q * Q);
            buffers->profile_accumulator[k] += mag;
        }
    }

    for (int k = 0; k < N_RANGE_BINS; k++) {
        float mean_magnitude = buffers->profile_accumulator[k] / num_integrated_chirps;
        buffers->mean_profile_db[k] = 20.0f * log10f(mean_magnitude + 1e-10f);
    }

    // Use moving average window size from config
    const int moving_avg_window_size = config->moving_avg_window_size;
    const int half_window = moving_avg_window_size / 2;
    float temp_profile[N_RANGE_BINS];

    for (int i = 0; i < N_RANGE_BINS; i++) {
        float sum = 0;
        int count = 0;
        for (int j = -half_window; j <= half_window; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < N_RANGE_BINS) {
                sum += buffers->mean_profile_db[idx];
                count++;
            }
        }
        temp_profile[i] = sum / count;
    }

    memcpy(buffers->mean_profile_db, temp_profile, N_RANGE_BINS * sizeof(float));
    
    int pad = config->presence_cfar_guards + config->presence_cfar_refs;
    for (int i = 0; i < N_RANGE_BINS; i++) {
        if (i < pad || i >= N_RANGE_BINS - pad) {
            buffers->cfar_threshold_db[i] = NAN;
            continue;
        }
        float noise_sum = 0;
        for (int j = i - pad; j < i - config->presence_cfar_guards; j++) noise_sum += buffers->mean_profile_db[j];
        for (int j = i + config->presence_cfar_guards + 1; j <= i + pad; j++) noise_sum += buffers->mean_profile_db[j];
        float noise_floor_estimate = noise_sum / (2.0f * config->presence_cfar_refs);
        buffers->cfar_threshold_db[i] = noise_floor_estimate + buffers->presence_cfar_bias_array[i];
    }
    
    uint8_t current_detections_count = 0;
    bool raw_target_found = false;
    buffers->history_write_idx = (buffers->history_write_idx + 1) % config->history_len;
    float* current_history_frame = buffers->presence_history[buffers->history_write_idx];
    for (int i = 0; i < N_RANGE_BINS; i++) {
        if (buffers->mean_profile_db[i] > buffers->cfar_threshold_db[i]) {
            raw_target_found = true;
            if (current_detections_count < MAX_DETECTIONS_PER_FRAME) {
                current_history_frame[current_detections_count++] = (float)i * R_MAX_M / N_RANGE_BINS;
            }
        }
    }
    buffers->history_detection_counts[buffers->history_write_idx] = current_detections_count;
    for (int i = current_detections_count; i < MAX_DETECTIONS_PER_FRAME; i++) current_history_frame[i] = 0.0f;
    
    bool confirmed = is_detection_persistent(buffers, config);
    
    // Set the raw and confirmed states. The occupancy task will handle the rest.
    radar_config_set_motion_detected(raw_target_found);
    radar_config_set_presence_confirmed(confirmed);

    ESP_LOGI(TAG, "Frame %lu | Raw Detections: %d | Presence Confirmed: %s | Occupancy: %s",
             frame_count, current_detections_count, confirmed ? "YES" : "No", radar_config_get_occupancy_state_safe() ? "YES" : "No");

    if (config->enable_uart_plotting) {
        printf("---PLOT_START---\n");
        printf("RANGE_PROFILE_DB:");
        for (int i = 0; i < N_RANGE_BINS; i++) {
            printf("%.2f%s", buffers->mean_profile_db[i], (i == N_RANGE_BINS - 1) ? "" : ",");
        }
        printf("\n");
        printf("CFAR_THRESHOLD_DB:");
        for (int i = 0; i < N_RANGE_BINS; i++) {
            if (isnan(buffers->cfar_threshold_db[i])) {
                 printf("nan%s", (i == N_RANGE_BINS - 1) ? "" : ",");
            } else {
                 printf("%.2f%s", buffers->cfar_threshold_db[i], (i == N_RANGE_BINS - 1) ? "" : ",");
            }
        }
        printf("\n");
        
        printf("DETECTIONS_RANGES_M:");
        // Only print detections if presence is confirmed for this frame
        if (confirmed) {
            bool first_det = true;
            float* latest_detections = buffers->presence_history[buffers->history_write_idx];
            uint8_t latest_count = buffers->history_detection_counts[buffers->history_write_idx];

            for (int i = 0; i < latest_count; i++) {
                 if (!first_det) {
                    printf(",");
                }
                printf("%.4f", latest_detections[i]);
                first_det = false;
            }
        }
        printf("\n");
        
        // Add the new occupancy state to the plot data
        printf("OCCUPANCY_STATE:%d\n", radar_config_get_occupancy_state_safe());

        printf("---PLOT_END---\n");
    }
}
