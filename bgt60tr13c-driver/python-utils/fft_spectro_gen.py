import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fft2, fftshift
from scipy.constants import c
from matplotlib.gridspec import GridSpec
from scipy.ndimage import label
import time
from collections import deque # To maintain detection history

# =============================================================================
# Algorithm & Tuning Parameters
# =============================================================================
# --- General ---
HIGH_RES_FFT_LEN = 1024 # FFT length for the high-resolution range profile
NUM_FRAMES_TO_AVERAGE = 1 # Number of frames to average for smoothing

# --- Stationary Target Detection ---
BACKGROUND_ALPHA = 0.05 # Learning rate for background model (lower is slower)
RECALIBRATION_INTERVAL_S = 60 # How often to force a background reset

# --- 1D CFAR (Presence Detection) ---
PRESENCE_CFAR_GUARDS = 2 # Guard cells for stationary detection
PRESENCE_CFAR_REFS = 5   # Reference (training) cells
PRESENCE_CFAR_BIAS = 1.25 # Sensitivity bias in dB (higher is less sensitive)

# --- 2D CFAR (Motion Detection) ---
MOTION_CFAR_GUARDS = 4 # Guard cells for motion detection
MOTION_CFAR_REFS = 6   # Reference (training) cells
MOTION_CFAR_PFA = 0.01 # Probability of False Alarm

# --- Temporal Filter (False Alarm Rejection) ---
HISTORY_LEN = 3 # Number of frames to look back on
MIN_DETECTIONS_IN_HISTORY = 2 # How many detections are needed to confirm a real target
MAX_RANGE_DIFF_M = 2.0 # Max range difference between detections in history to be considered the same target

# =============================================================================
# Core Functions (CFAR, Temporal Check)
# =============================================================================
def cfar_fast_1d(x_db, num_ref_cells, num_guard_cells, bias_db):
    pad = int(num_ref_cells + num_guard_cells)
    if x_db.size < (2 * pad + 1): return np.full_like(x_db, np.nan)
    all_windows = np.lib.stride_tricks.sliding_window_view(x_db, (num_ref_cells * 2) + (num_guard_cells * 2) + 1)
    indices_to_delete = np.arange(num_ref_cells, num_ref_cells + (num_guard_cells * 2) + 1)
    reference_cells = np.delete(all_windows, indices_to_delete, axis=1)
    noise_floor_estimate = np.mean(reference_cells, axis=1)
    thresholds = np.pad(noise_floor_estimate, (pad, pad), constant_values=np.nan)
    return thresholds + bias_db

def ca_cfar_2d(rd_map, training_cells, guard_cells, pfa):
    num_ranges, num_doppler_bins = rd_map.shape
    num_training_cells = 2 * training_cells
    alpha = num_training_cells * (pfa ** (-1 / num_training_cells) - 1)
    detection_mask = np.zeros_like(rd_map, dtype=bool)
    for r in range(num_ranges):
        for i in range(training_cells + guard_cells, num_doppler_bins - (training_cells + guard_cells)):
            cut = rd_map[r, i]
            noise_floor_lead = np.sum(rd_map[r, i - training_cells - guard_cells : i - guard_cells])
            noise_floor_lag = np.sum(rd_map[r, i + guard_cells + 1 : i + guard_cells + training_cells + 1])
            noise_floor_avg = (noise_floor_lead + noise_floor_lag) / num_training_cells
            threshold = alpha * noise_floor_avg
            if cut > threshold: detection_mask[r, i] = True
    return detection_mask

def check_temporal_filter(history, max_range_diff):
    """Checks if there are enough recent detections within a certain range tolerance."""
    # Get all frames in history that had at least one detection
    frames_with_detections = [frame_detections for frame_detections in history if frame_detections]
    
    # Check if we have enough recent detections
    if len(frames_with_detections) < MIN_DETECTIONS_IN_HISTORY:
        return False
    
    # Check if any detection in the most recent frame is close to any detection in other recent frames
    latest_detections = frames_with_detections[-1]
    past_detections_flat = [item for sublist in frames_with_detections[:-1] for item in sublist]

    for latest_det in latest_detections:
        for past_det in past_detections_flat:
            if abs(latest_det - past_det) <= max_range_diff:
                return True # Found a match, confirm detection
    return False

# =============================================================================
# Radar Parameters and Setup
# =============================================================================
f_low = 59479900000; f_high = 61479900000; f_bandwidth = abs(f_high-f_low)
Tc = 0.00005738; shape_end_delay = 0.000690; sample_rate = 2352941
PRT = shape_end_delay + Tc; samples_per_chirp = 128; chirps_per_frame = 64

r_max = (sample_rate * c * Tc) / (4 * f_bandwidth)
v_max =  c / (2 * f_bandwidth * PRT)
high_res_range_axis = np.linspace(0, r_max, HIGH_RES_FFT_LEN // 2)
low_res_range_axis = np.linspace(0, r_max, samples_per_chirp // 2)

ser = serial.Serial('/dev/tty.SLAB_USBtoUART2', 921600)

# --- Plotting Setup ---
plt.ion()
fig, (ax_main, ax_presence) = plt.subplots(2, 1, figsize=(10, 10), gridspec_kw={'height_ratios': [3, 2]})

im = ax_main.imshow(np.zeros((samples_per_chirp // 2, chirps_per_frame)), aspect='auto', 
                    extent=[-v_max/2, v_max/2, 0, r_max], origin='lower', vmin=-50, vmax=30)
fig.colorbar(im, ax=ax_main)
ax_main.set_title("Motion Detection Map"); ax_main.set_xlabel("Velocity (m/s)"); ax_main.set_ylabel("Range (m)")

ax_presence.set_title("Presence Detection (Stationary Targets)"); ax_presence.set_xlabel("Range (m)")
ax_presence.set_ylabel("Magnitude (dB)"); ax_presence.set_xlim([0, r_max]); ax_presence.grid(True)
line_stationary_profile, = ax_presence.plot(high_res_range_axis, np.zeros_like(high_res_range_axis), c='g', lw=1.0, label='Stationary Profile')
line_stationary_thresh, = ax_presence.plot(high_res_range_axis, np.zeros_like(high_res_range_axis), c='y', lw=1.5, ls='--', label='CFAR Threshold')
line_stationary_targets, = ax_presence.plot([], [], 'ro', markersize=8, label='Presence Detected')
ax_presence.legend(loc="upper right")

# --- Processing Variables ---
accumulated_rd_map = np.zeros((samples_per_chirp // 2, chirps_per_frame))
frame_count = 0; background_model = None
last_recalibration_time = time.time()
presence_history = deque(maxlen=HISTORY_LEN)
motion_history = deque(maxlen=HISTORY_LEN)

# =============================================================================
# Main Processing Loop
# =============================================================================
while True:
    try:
        if time.time() - last_recalibration_time > RECALIBRATION_INTERVAL_S:
            print(f"\n--- Recalibrating background model ({RECALIBRATION_INTERVAL_S}s elapsed) ---\n")
            background_model = None; last_recalibration_time = time.time()

        line = ser.readline().decode().strip()
        if not line.startswith('Frame'): continue
        
        data_str = line.split(':', 1)[1].strip()
        data = eval(data_str) if '[' in data_str else None
        if not data or len(data) < chirps_per_frame * samples_per_chirp: continue
        
        raw_data = np.array([float(x) for x in data], dtype=np.float32).reshape((chirps_per_frame, samples_per_chirp))

        # --- PATH A: Moving Target Processing ---
        moving_target_data = raw_data - np.mean(raw_data, axis=0)
        
        # --- PATH B: Stationary Target Processing ---
        if background_model is None: background_model = np.copy(raw_data)
        stationary_target_data = raw_data - background_model
        background_model = (1 - BACKGROUND_ALPHA) * background_model + BACKGROUND_ALPHA * raw_data
        
        # --- Windowing and FFT Processing ---
        windowed_moving_data = moving_target_data * np.hanning(samples_per_chirp)
        windowed_stationary_data = stationary_target_data * np.hanning(samples_per_chirp)
        
        rd_map_2d = fft2(windowed_moving_data.T, s=(samples_per_chirp, chirps_per_frame))
        rd_moving = fftshift(np.abs(rd_map_2d), axes=(1,))[0:samples_per_chirp // 2, :]
        
        accumulated_rd_map += rd_moving; frame_count += 1

        if frame_count >= NUM_FRAMES_TO_AVERAGE:
            averaged_rd_map = accumulated_rd_map / NUM_FRAMES_TO_AVERAGE

            # --- Motion Detection Logic ---
            moving_detection_mask = ca_cfar_2d(averaged_rd_map, MOTION_CFAR_REFS, MOTION_CFAR_GUARDS, MOTION_CFAR_PFA)
            labeled_array, num_features = label(moving_detection_mask)
            
            current_motion_ranges = []
            if num_features > 0:
                for i in range(1, num_features + 1):
                    cluster_indices = np.argwhere(labeled_array == i)
                    center_range_bin = cluster_indices[:, 0].mean()
                    current_motion_ranges.append(center_range_bin * (r_max / (samples_per_chirp / 2)))
            motion_history.append(current_motion_ranges)
            motion_confirmed = check_temporal_filter(motion_history, MAX_RANGE_DIFF_M)

            rd_log = 20 * np.log10(averaged_rd_map + 1e-10)
            vmin, vmax = im.get_clim()
            motion_map = np.full_like(rd_log, vmin)
            if motion_confirmed: motion_map[moving_detection_mask] = rd_log[moving_detection_mask]
            im.set_data(motion_map)

            # --- Presence Detection Logic ---
            range_ffts_stationary = fft(windowed_stationary_data, n=HIGH_RES_FFT_LEN, axis=1)[:, :HIGH_RES_FFT_LEN // 2]
            stationary_profile_db = 20 * np.log10(np.mean(np.abs(range_ffts_stationary), axis=0) + 1e-10)
            cfar_threshold_db = cfar_fast_1d(stationary_profile_db, PRESENCE_CFAR_REFS, PRESENCE_CFAR_GUARDS, PRESENCE_CFAR_BIAS)
            stationary_target_indices = np.where(stationary_profile_db > cfar_threshold_db)[0]
            
            current_presence_ranges = high_res_range_axis[stationary_target_indices].tolist()
            presence_history.append(current_presence_ranges)
            presence_confirmed = check_temporal_filter(presence_history, MAX_RANGE_DIFF_M)

            line_stationary_profile.set_ydata(stationary_profile_db)
            line_stationary_thresh.set_ydata(cfar_threshold_db)
            if presence_confirmed:
                line_stationary_targets.set_data(current_presence_ranges, stationary_profile_db[stationary_target_indices])
            else:
                line_stationary_targets.set_data([], [])

            y_min, y_max = np.nanmin(stationary_profile_db), np.nanmax(stationary_profile_db)
            ax_presence.set_ylim(y_min - 5 if np.isfinite(y_min) else 0, y_max + 5 if np.isfinite(y_max) else 60)
            
            # --- Print Final Status ---
            print(f"\rFrame Status -> Presence: {'YES' if presence_confirmed else 'No '} | Motion: {'YES' if motion_confirmed else 'No '}", end="")

            accumulated_rd_map.fill(0); frame_count = 0

        fig.tight_layout(); fig.canvas.draw_idle(); plt.pause(0.01)
        
    except KeyboardInterrupt: print("\nStopping..."); break
    except Exception as e: print(f"An error occurred: {e}"); continue