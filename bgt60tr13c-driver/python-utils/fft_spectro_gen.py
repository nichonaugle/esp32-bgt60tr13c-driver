import serial
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft
import time

# =============================================================================
# Algorithm & Tuning Parameters
# =============================================================================
# --- General ---
HIGH_RES_FFT_LEN = 1024       # FFT length for the high-resolution range profile
C = 299792458.0               # Speed of Light (m/s)

# --- Presence Detection (CFAR) ---
BACKGROUND_ALPHA = 0.05       # Learning rate for background model (lower is slower)
RECALIBRATION_INTERVAL_S = 60 # How often to force a background reset
NEAR_RANGE_BIAS_DB = 3.0      # CFAR bias at close range (less sensitive)
FAR_RANGE_BIAS_DB = 1.25      # CFAR bias at far range (more sensitive)
PRESENCE_CFAR_GUARDS = 6      # Guard cells to handle wide targets
PRESENCE_CFAR_REFS = 4        # Reference cells for noise estimation

# --- Temporal Filter (False Alarm Rejection) ---
HISTORY_LEN = 3               # Number of frames to look back on
MIN_DETECTIONS_IN_HISTORY = 2 # Detections needed to confirm a target
MAX_RANGE_DIFF_M = 2.0        # Max range difference for a target in history

# =============================================================================
# Core Functions
# =============================================================================
def cfar_1d(x_db, num_ref_cells, num_guard_cells, bias_array):
    """Calculates the 1D CFAR threshold using numpy's stride tricks for efficiency."""
    pad = num_ref_cells + num_guard_cells
    if x_db.size < (2 * pad + 1):
        return np.full_like(x_db, np.nan)
    all_windows = np.lib.stride_tricks.sliding_window_view(x_db, (2 * pad + 1))
    noise_floor_lead = np.sum(all_windows[:, :num_ref_cells], axis=1)
    noise_floor_lag = np.sum(all_windows[:, -num_ref_cells:], axis=1)
    noise_floor_estimate = (noise_floor_lead + noise_floor_lag) / (2 * num_ref_cells)
    thresholds = np.pad(noise_floor_estimate, (pad, pad), constant_values=np.nan)
    return thresholds + bias_array

def is_detection_persistent(history, max_range_diff):
    """Checks if a detection has been present in enough recent frames to be considered valid."""
    frames_with_detections = [frame for frame in history if frame]
    if len(frames_with_detections) < MIN_DETECTIONS_IN_HISTORY:
        return False
    latest_detections = frames_with_detections[-1]
    for latest_det in latest_detections:
        for past_frame in frames_with_detections[:-1]:
            for past_det in past_frame:
                if abs(latest_det - past_det) <= max_range_diff:
                    return True # Found a match, confirm detection
    return False

# =============================================================================
# Radar Parameters & Pre-computation
# =============================================================================
# --- FMCW Parameters ---
f_bandwidth = 2000000000.0 # 2 GHz
Tc = 0.00005738
sample_rate = 2352941
samples_per_chirp = 128
chirps_per_frame = 64

# --- Calculated Metrics ---
r_max = (sample_rate * C * Tc) / (4 * f_bandwidth)
high_res_range_axis = np.linspace(0, r_max, HIGH_RES_FFT_LEN // 2)
hanning_window = np.hanning(samples_per_chirp)
presence_cfar_bias_array = np.linspace(
    NEAR_RANGE_BIAS_DB, FAR_RANGE_BIAS_DB, HIGH_RES_FFT_LEN // 2
)

# =============================================================================
# Setup & Main Loop
# =============================================================================
# --- Serial Port and Processing Variables ---
try:
    ser = serial.Serial('/dev/tty.SLAB_USBtoUART2', 921600, timeout=1)
    print("STATUS: Serial port opened. Waiting for data...")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port: {e}")
    exit()

background_model = None
last_recalibration_time = time.time()
presence_history = [] # Using a standard list instead of deque
profile_accumulator = np.zeros(HIGH_RES_FFT_LEN // 2, dtype=np.float32)

# --- Plotting Setup (For Python visualization only, not part of C port) ---
plt.ion()
fig, ax_presence = plt.subplots(1, 1, figsize=(12, 6))
ax_presence.set_title("Presence Detection"); ax_presence.set_xlabel("Range (m)"); ax_presence.set_ylabel("Magnitude (dB)")
ax_presence.set_xlim([0, r_max]); ax_presence.grid(True)
# FIX: Initialize plot lines with correctly dimensioned placeholder data
line_profile, = ax_presence.plot(high_res_range_axis, np.zeros_like(high_res_range_axis), 'g-', lw=1.0, label='Range Profile')
line_thresh, = ax_presence.plot(high_res_range_axis, np.zeros_like(high_res_range_axis), 'y--', lw=1.5, label='CFAR Threshold')
line_targets, = ax_presence.plot([], [], 'ro', markersize=8, label='Presence Detected')
ax_presence.legend(loc="upper right")
print("STATUS: Plot configured. Starting main loop.")

# --- Main Processing Loop ---
while True:
    try:
        if time.time() - last_recalibration_time > RECALIBRATION_INTERVAL_S:
            print("\n--- Recalibrating background model ---")
            background_model = None; last_recalibration_time = time.time()

        line = ser.readline()
        if not line or not line.startswith(b'Frame'): continue
        
        data_start = line.find(b'[') + 1
        data_end = line.rfind(b']')
        if data_start == 0 or data_end == -1: continue
        
        raw_data = np.fromstring(line[data_start:data_end].decode(), sep=',', dtype=np.float32)
        if raw_data.size < chirps_per_frame * samples_per_chirp: continue
        raw_data = raw_data.reshape((chirps_per_frame, samples_per_chirp))

        if background_model is None: background_model = np.copy(raw_data)
        
        stationary_data = raw_data - background_model
        background_model = (1 - BACKGROUND_ALPHA) * background_model + BACKGROUND_ALPHA * raw_data
        stationary_data *= hanning_window
        
        profile_accumulator.fill(0)
        for i in range(chirps_per_frame):
            fft_result = fft(stationary_data[i,:], n=HIGH_RES_FFT_LEN)
            profile_accumulator += np.abs(fft_result[:HIGH_RES_FFT_LEN // 2])
            
        mean_profile = profile_accumulator / chirps_per_frame
        profile_db = 20 * np.log10(mean_profile + 1e-10)
        
        threshold_db = cfar_1d(profile_db, PRESENCE_CFAR_REFS, PRESENCE_CFAR_GUARDS, presence_cfar_bias_array)
        
        detected_indices = np.where(profile_db > threshold_db)[0]
        detected_ranges = high_res_range_axis[detected_indices].tolist()
        
        presence_history.append(detected_ranges)
        if len(presence_history) > HISTORY_LEN:
            presence_history.pop(0) # Manually maintain history list size

        is_confirmed = is_detection_persistent(presence_history, MAX_RANGE_DIFF_M)

        print(f"\rPresence Detected: {'YES' if is_confirmed else 'No '}", end="")
        
        line_profile.set_ydata(profile_db)
        line_thresh.set_ydata(threshold_db)
        if is_confirmed:
            line_targets.set_data(detected_ranges, profile_db[detected_indices])
        else:
            line_targets.set_data([], [])

        y_min, y_max = np.nanmin(profile_db[10:-10]), np.nanmax(profile_db)
        if np.isfinite(y_min) and np.isfinite(y_max):
            ax_presence.set_ylim(y_min - 5, y_max + 10)
        
        plt.pause(0.001)
        
    except KeyboardInterrupt:
        print("\nStopping..."); break
    except Exception: continue