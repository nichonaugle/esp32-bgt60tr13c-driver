import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm  # Import colormap library
from scipy import signal # For window functions
from scipy.constants import c # Speed of light
from numpy.fft import fft, fftshift # FFT functions
from numpy.lib.stride_tricks import sliding_window_view # For fast CFAR
import warnings
import serial
import time
import re

# --- Suppress DeprecationWarning from sliding_window_view ---
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# --- Serial Port Configuration ---
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'  # <<< --- CHANGE THIS TO YOUR ESP32's SERIAL PORT
BAUD_RATE = 921600  # Match the ESP32's baud rate
SERIAL_TIMEOUT_S = 0.1  # Serial read timeout in seconds

# --- Radar & System Parameter Configuration (BGT60TR13C based) ---
# These should match the configuration in your test.c or radar driver
F_START_HZ = 60e9  # Start frequency of the chirp (Hz)
F_END_HZ = 62e9  # End frequency of the chirp (Hz)
TC_S = 0.0001935   # Chirp Repetition Time or Effective Chirp Duration (s)

M_CHIRPS = 16     # Number of chirps per frame (NUM_CHIRPS_PER_FRAME in test.c)
N_SAMPLES_PER_CHIRP = 256 # Number of ADC samples per chirp (NUM_SAMPLES_PER_CHIRP in test.c)
NUM_RX_ANTENNAS = 3 # Number of RX antennas used by the ESP32 (NUM_RX_ANTENNAS in test.c)
ANTENNA_INDEX_TO_DISPLAY = 1 # 0 for Rx1, 1 for Rx2, 2 for Rx3

EXPECTED_SAMPLES_IN_FLAT_ARRAY = M_CHIRPS * N_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS

# ADC Sampling Frequency (FS_HZ)
FS_HZ = N_SAMPLES_PER_CHIRP / TC_S
print(f"ADC Sampling Frequency (FS_HZ): {FS_HZ / 1e6:.2f} MHz (derived)")

TS_S = 1 / FS_HZ # Sampling period (s)

# Calculated Radar Parameters
B_HZ = F_END_HZ - F_START_HZ  # Bandwidth (Hz)
FC_HZ = (F_START_HZ + F_END_HZ) / 2.0  # Center frequency (Hz)
LAMBDA_M = c / FC_HZ  # Wavelength (m)
CHIRP_RATE_HZ_PER_S = B_HZ / TC_S  # Chirp rate (Hz/s)

print(f"--- Radar Parameters (from Python script) ---")
print(f"Bandwidth (B): {B_HZ / 1e9:.2f} GHz")
print(f"Center Frequency (fc): {FC_HZ / 1e9:.2f} GHz")
print(f"Wavelength (lambda): {LAMBDA_M:.4f} m")
print(f"Chirp Duration (Tc): {TC_S * 1e6:.2f} us")
print(f"Number of Chirps (M): {M_CHIRPS}")
print(f"Samples per Chirp (N): {N_SAMPLES_PER_CHIRP}")

# --- FFT and Resolution Parameters ---
RANGE_RESOLUTION_M = c / (2 * B_HZ)
print(f"Calculated Range Resolution: {RANGE_RESOLUTION_M*100:.2f} cm")
R_MAX_M = (FS_HZ / 2) * c * TC_S / (2 * B_HZ)
print(f"Calculated Max Unambiguous Range: {R_MAX_M:.2f} m")

VELOCITY_RESOLUTION_MPS = LAMBDA_M / (2 * M_CHIRPS * TC_S)
print(f"Calculated Velocity Resolution: {VELOCITY_RESOLUTION_MPS:.2f} m/s")
V_MAX_MPS = LAMBDA_M / (4 * TC_S)
print(f"Calculated Max Unambiguous Velocity: +/- {V_MAX_MPS:.2f} m/s")

RANGE_FFT_LEN = N_SAMPLES_PER_CHIRP * 4
DOPPLER_FFT_LEN = M_CHIRPS * 4

# --- CFAR Target Detection Function (adapted from notebook example) ---
def cfar_fast_db(x_db, num_ref_cells, num_guard_cells, bias_db, method=np.mean):
    """
    Fast, vectorized CFAR implementation for log-magnitude (dB) data.
    """
    pad = int(num_ref_cells + num_guard_cells)
    
    # Use sliding_window_view to create windows of the input signal
    all_windows = sliding_window_view(x_db, (num_ref_cells * 2) + (num_guard_cells * 2) + 1)
    
    # Remove guard cells and the "cell under test" (CUT) from each window
    # Indices to delete are the guard cells and the CUT in the middle of the window
    indices_to_delete = np.arange(num_ref_cells, num_ref_cells + (num_guard_cells * 2) + 1)
    reference_cells = np.delete(all_windows, indices_to_delete, axis=1)
    
    # Apply the specified method (e.g., np.mean) to the reference cells for each window
    noise_floor_estimate = method(reference_cells, axis=1)
    
    # Pad the result to match the original signal length
    thresholds = np.pad(noise_floor_estimate, (pad, pad), constant_values=(np.nan, np.nan))
    
    # Add the bias (offset) in dB
    return thresholds + bias_db

# --- Data Acquisition and Parsing ---
ser = None
log_file = None # File handle for logging

def connect_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_S)
        print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
        time.sleep(1) # Allow ESP32 to initialize/reset
        ser.reset_input_buffer()
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return False

def parse_frame_data_from_serial(ser_conn):
    if not ser_conn or not ser_conn.is_open:
        print("Serial port not open.")
        return None, None

    try:
        line = ser_conn.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return None, None

        if line.startswith("Frame") and log_file:
            log_file.write(line + "\n")

        match = re.match(r"Frame (\d+): \[(.*)\]", line)
        if match:
            frame_num_str, data_str = match.groups()
            frame_num = int(frame_num_str)
            
            try:
                flat_data_1d = np.array([float(x) for x in data_str.split(', ') if x], dtype=np.float32)
            except ValueError:
                print(f"Warning: ValueError parsing data string: {data_str[:100]}...")
                return None, None

            if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                antenna_data_flat = flat_data_1d[ANTENNA_INDEX_TO_DISPLAY::NUM_RX_ANTENNAS]
                
                if antenna_data_flat.shape[0] == M_CHIRPS * N_SAMPLES_PER_CHIRP:
                    cpi_matrix = antenna_data_flat.reshape((M_CHIRPS, N_SAMPLES_PER_CHIRP))
                    return cpi_matrix, frame_num
                else:
                    print(f"Data length mismatch for selected antenna: Exp {M_CHIRPS * N_SAMPLES_PER_CHIRP}, Got {antenna_data_flat.shape[0]}")
                    return None, None
            else:
                print(f"Data length mismatch: Exp {EXPECTED_SAMPLES_IN_FLAT_ARRAY}, Got {len(flat_data_1d)}")
                return None, None
        else:
            return None, None
            
    except Exception as e:
        print(f"Error in parse_frame_data_from_serial: {e}")
        import traceback
        traceback.print_exc()
        return None, None

# --- Plotting Setup ---
plt.ion()
# Add a 4th subplot for the CFAR results
fig, (ax_raw, ax_range, ax_doppler, ax_cfar) = plt.subplots(4, 1, figsize=(10, 24))

# --- Raw Data Plot Setup ---
initial_raw_data = np.zeros((M_CHIRPS, N_SAMPLES_PER_CHIRP))
img_raw_map = ax_raw.imshow(initial_raw_data, aspect='auto', origin='lower', cmap='viridis')
fig.colorbar(img_raw_map, ax=ax_raw, label='ADC Value')
ax_raw.set_xlabel("Sample Number (Time)")
ax_raw.set_ylabel("Chirp Number")
ax_raw.set_title("Raw ADC Data Matrix (Live)")

# --- Range Profile Plot Setup for All Chirps ---
range_axis_plot_m = np.linspace(0, R_MAX_M, RANGE_FFT_LEN // 2)
PLOT_R_MAX_DISPLAY_M = min(R_MAX_M, 5.0)
ax_range.set_xlabel("Range (m)")
ax_range.set_ylabel("Magnitude (dB)")
ax_range.set_title("Range Profile per Chirp")
ax_range.set_xlim(0, PLOT_R_MAX_DISPLAY_M)
ax_range.grid(True)
colors = cm.jet(np.linspace(0, 1, M_CHIRPS))
lines_range_profile = []
for i in range(M_CHIRPS):
    line, = ax_range.plot(range_axis_plot_m, np.zeros(RANGE_FFT_LEN // 2), color=colors[i], lw=0.75)
    lines_range_profile.append(line)
sm = plt.cm.ScalarMappable(cmap=cm.jet, norm=plt.Normalize(vmin=0, vmax=M_CHIRPS - 1))
fig.colorbar(sm, ax=ax_range, label='Chirp Number')

# --- Range-Doppler Plot Setup ---
velocity_axis_plot_mps = np.linspace(-V_MAX_MPS, V_MAX_MPS, DOPPLER_FFT_LEN)
initial_img_data = np.zeros((DOPPLER_FFT_LEN, RANGE_FFT_LEN // 2))
img_range_doppler = ax_doppler.imshow(initial_img_data, aspect='auto', origin='lower', extent=[0, R_MAX_M, velocity_axis_plot_mps[0], velocity_axis_plot_mps[-1]], cmap='jet')
fig.colorbar(img_range_doppler, ax=ax_doppler, label='Magnitude (dB)')
ax_doppler.set_xlabel("Range (m)")
ax_doppler.set_ylabel("Velocity (m/s)")
ax_doppler.set_title("Range-Doppler Spectrum (Live Data)")
ax_doppler.set_xlim(0, PLOT_R_MAX_DISPLAY_M)
ax_doppler.set_ylim(-V_MAX_MPS, V_MAX_MPS)

# --- CFAR Target Plot Setup ---
ax_cfar.set_xlabel("Range (m)")
ax_cfar.set_ylabel("Magnitude (dB)")
ax_cfar.set_title("Averaged Range Profile & CFAR Detections")
ax_cfar.set_xlim(0, PLOT_R_MAX_DISPLAY_M)
ax_cfar.grid(True)
line_avg_range, = ax_cfar.plot(range_axis_plot_m, np.zeros(RANGE_FFT_LEN // 2), c='b', lw=1.0, label='Avg. Range Profile')
line_cfar_thresh, = ax_cfar.plot(range_axis_plot_m, np.zeros(RANGE_FFT_LEN // 2), c='y', lw=1.5, ls='--', label='CFAR Threshold')
line_targets, = ax_cfar.plot([], [], 'ro', markersize=8, label='Detected Targets')
ax_cfar.legend(loc="upper right")


plt.tight_layout(pad=3.0)
fig.canvas.draw_idle()
plt.show(block=False)

# --- Main Loop ---
if not connect_serial():
    print("Exiting due to serial connection failure.")
    exit()

try:
    log_file = open("radar_frames.log", "w")
    print("--- Logging full frames to radar_frames.log ---")
except IOError as e:
    print(f"Error opening log file 'radar_frames.log': {e}")
    log_file = None

print("\n--- Starting Real-Time Radar Processing ---")
try:
    while plt.fignum_exists(fig.number):
        cpi_data_matrix, frame_num = parse_frame_data_from_serial(ser)

        if cpi_data_matrix is not None and frame_num is not None:
            # --- Update Raw Data Plot ---
            img_raw_map.set_data(cpi_data_matrix)
            raw_min, raw_max = np.min(cpi_data_matrix), np.max(cpi_data_matrix)
            img_raw_map.set_clim(vmin=raw_min, vmax=raw_max)
            ax_raw.set_title(f"Raw ADC Data Matrix (Frame: {frame_num}, Ant: {ANTENNA_INDEX_TO_DISPLAY+1})")
            
            # --- Windowing ---
            range_window = signal.windows.hamming(N_SAMPLES_PER_CHIRP)
            cpi_data_windowed_range = cpi_data_matrix * range_window[np.newaxis, :]
            doppler_window = signal.windows.hamming(M_CHIRPS)
            cpi_data_windowed_both = cpi_data_windowed_range * doppler_window[:, np.newaxis]

            # --- Range FFT ---
            range_fft_result = fft(cpi_data_windowed_both, n=RANGE_FFT_LEN, axis=1)

            # --- Update Range Profile Plot ---
            range_profiles_abs = np.abs(range_fft_result[:, :RANGE_FFT_LEN//2])
            range_profiles_db = 20 * np.log10(range_profiles_abs + 1e-9)
            for i in range(M_CHIRPS):
                lines_range_profile[i].set_ydata(range_profiles_db[i, :])
            max_db_val = np.max(range_profiles_db)
            ax_range.set_ylim(0, max_db_val + 5 if max_db_val > -np.inf else 10)
            ax_range.set_title(f"Range Profile per Chirp (Frame: {frame_num}, Ant: {ANTENNA_INDEX_TO_DISPLAY+1})")

            # --- CFAR Processing on Averaged Range Profile ---
            avg_range_profile_db = np.mean(range_profiles_db, axis=0)
            
            # CFAR Parameters - these can be tuned
            NUM_GUARD_CELLS = 4
            NUM_REF_CELLS = 10
            CFAR_BIAS_DB = 12.0 # Bias in dB to be added to the noise floor estimate

            cfar_threshold_db = cfar_fast_db(
                avg_range_profile_db,
                NUM_REF_CELLS,
                NUM_GUARD_CELLS,
                bias_db=CFAR_BIAS_DB
            )
            # Find indices where the signal is above the CFAR threshold
            target_indices = np.where(avg_range_profile_db > cfar_threshold_db)[0]

            # --- Update CFAR Plot ---
            line_avg_range.set_ydata(avg_range_profile_db)
            line_cfar_thresh.set_ydata(cfar_threshold_db)
            
            # Plot detected targets as red circles
            if len(target_indices) > 0:
                target_ranges_m = range_axis_plot_m[target_indices]
                target_magnitudes_db = avg_range_profile_db[target_indices]
                line_targets.set_data(target_ranges_m, target_magnitudes_db)
            else:
                line_targets.set_data([], []) # Clear targets if none are found

            y_min_cfar = np.nanmin(avg_range_profile_db)
            y_max_cfar = np.nanmax(avg_range_profile_db)
            ax_cfar.set_ylim(max(y_min_cfar - 5, 0), y_max_cfar + 5 if y_max_cfar > -np.inf else 10)
            ax_cfar.set_title(f"CFAR Detections (Frame: {frame_num}, Ant: {ANTENNA_INDEX_TO_DISPLAY+1})")


            # --- Range-Doppler FFT Processing ---
            range_doppler_fft_result = fft(range_fft_result, n=DOPPLER_FFT_LEN, axis=0)
            range_doppler_spectrum_shifted = fftshift(range_doppler_fft_result, axes=0)
            range_doppler_spectrum_abs = np.abs(range_doppler_spectrum_shifted)
            range_doppler_spectrum_db = 20 * np.log10(range_doppler_spectrum_abs + 1e-9)
            range_doppler_to_plot_db = range_doppler_spectrum_db[:, :RANGE_FFT_LEN//2]

            # --- Update Range-Doppler Plot ---
            img_range_doppler.set_data(range_doppler_to_plot_db)
            current_max_db = np.max(range_doppler_to_plot_db)
            img_range_doppler.set_clim(vmin=current_max_db - 40, vmax=current_max_db)
            ax_doppler.set_title(f"Range-Doppler Spectrum (Frame: {frame_num}, Ant: {ANTENNA_INDEX_TO_DISPLAY+1})")
            
            # --- Draw All Plots ---
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Program stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")
    import traceback
    traceback.print_exc()
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    if log_file:
        log_file.close()
        print("Log file 'radar_frames.log' closed.")
    plt.ioff()
    print("--- Script Finished ---")
    if plt.fignum_exists(fig.number):
         print("Close the plot window to exit completely.")
         plt.show(block=True)