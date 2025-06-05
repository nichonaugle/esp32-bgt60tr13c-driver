import numpy as np
import matplotlib.pyplot as plt
from scipy import signal # For window functions
from scipy.constants import c # Speed of light
from numpy.fft import fft, fftshift # FFT functions (fft2 not strictly needed for 2x1D FFTs)
import serial
import time
import re

# --- Serial Port Configuration ---
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'  # <<< --- CHANGE THIS TO YOUR ESP32's SERIAL PORT
BAUD_RATE = 921600  # Match the ESP32's baud rate
SERIAL_TIMEOUT_S = 0.1  # Serial read timeout in seconds

# --- Radar & System Parameter Configuration (BGT60TR13C based) ---
# These should match the configuration in your test.c or radar driver
F_START_HZ = 58e9  # Start frequency of the chirp (Hz)
F_END_HZ = 62e9    # End frequency of the chirp (Hz)
TC_S = 0.0005911   # Chirp Repetition Time or Effective Chirp Duration (s)

M_CHIRPS = 64      # Number of chirps per frame (NUM_CHIRPS_PER_FRAME in test.c)
N_SAMPLES_PER_CHIRP = 128 # Number of ADC samples per chirp (NUM_SAMPLES_PER_CHIRP in test.c)
NUM_RX_ANTENNAS = 3 # Number of RX antennas used by the ESP32 (NUM_RX_ANTENNAS in test.c)
ANTENNA_INDEX_TO_DISPLAY = 0 # 0 for Rx1, 1 for Rx2, 2 for Rx3

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

# --- Data Acquisition and Parsing ---
ser = None
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
    """
    Reads a line from serial, parses it if it matches the frame format,
    and returns the data for the selected antenna, frame number, or None.
    Format: "Frame <frame_num>: [<sample1>, <sample2>, ..., <sampleN>]"
    """
    if not ser_conn or not ser_conn.is_open:
        print("Serial port not open.")
        return None, None

    try:
        line = ser_conn.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return None, None # Timeout or empty line

        match = re.match(r"Frame (\d+): \[(.*)\]", line)
        if match:
            frame_num_str, data_str = match.groups()
            frame_num = int(frame_num_str)
            
            try:
                # Split by comma and space, then convert to float
                flat_data_1d = np.array([float(x) for x in data_str.split(', ') if x], dtype=np.float32)
            except ValueError:
                print(f"Warning: ValueError parsing data string: {data_str[:100]}...")
                return None, None

            if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                # De-interleave for the selected antenna
                antenna_data_flat = flat_data_1d[ANTENNA_INDEX_TO_DISPLAY::NUM_RX_ANTENNAS]
                
                if antenna_data_flat.shape[0] == M_CHIRPS * N_SAMPLES_PER_CHIRP:
                    # Reshape to (M_CHIRPS, N_SAMPLES_PER_CHIRP)
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
fig, ax = plt.subplots(figsize=(10, 7))

range_axis_plot_m = np.linspace(0, R_MAX_M, RANGE_FFT_LEN // 2)
velocity_axis_plot_mps = np.linspace(-V_MAX_MPS, V_MAX_MPS, DOPPLER_FFT_LEN)

initial_img_data = np.zeros((DOPPLER_FFT_LEN, RANGE_FFT_LEN // 2))

img_range_doppler = ax.imshow(
    initial_img_data,
    aspect='auto',
    origin='lower',
    extent=[0, R_MAX_M, velocity_axis_plot_mps[0], velocity_axis_plot_mps[-1]],
    cmap='jet'
)
plt.colorbar(img_range_doppler, ax=ax, label='Magnitude (dB)')
ax.set_xlabel("Range (m)")
ax.set_ylabel("Velocity (m/s)")
ax.set_title("Range-Doppler Spectrum (Live Data)")

PLOT_R_MAX_DISPLAY_M = min(R_MAX_M, 5.0)
ax.set_xlim(0, PLOT_R_MAX_DISPLAY_M)
ax.set_ylim(-V_MAX_MPS, V_MAX_MPS)

plt.tight_layout()
fig.canvas.draw_idle()
plt.show(block=False)


# --- Main Loop ---
if not connect_serial():
    print("Exiting due to serial connection failure.")
    exit()

print("\n--- Starting Real-Time Radar Processing ---")
try:
    while plt.fignum_exists(fig.number): # Keep running as long as the plot window is open
        cpi_data_matrix, frame_num = parse_frame_data_from_serial(ser)

        if cpi_data_matrix is not None and frame_num is not None:
            # --- Windowing ---
            range_window = signal.windows.blackman(N_SAMPLES_PER_CHIRP)
            cpi_data_windowed_range = cpi_data_matrix * range_window[np.newaxis, :]

            doppler_window = signal.windows.blackman(M_CHIRPS)
            cpi_data_windowed_both = cpi_data_windowed_range * doppler_window[:, np.newaxis]

            # --- Range-Doppler FFT Processing ---
            # 1. Range FFT
            range_fft_result = fft(cpi_data_windowed_both, n=RANGE_FFT_LEN, axis=1)
            
            # 2. Doppler FFT
            range_doppler_fft_result = fft(range_fft_result, n=DOPPLER_FFT_LEN, axis=0)
            
            range_doppler_spectrum_shifted = fftshift(range_doppler_fft_result, axes=(0,1))
            range_doppler_spectrum_abs = np.abs(range_doppler_spectrum_shifted)
            range_doppler_spectrum_db = 20 * np.log10(range_doppler_spectrum_abs + 1e-9)
            
            # Select portion for plotting (Doppler x Range_positive_half)
            range_doppler_to_plot_db = range_doppler_spectrum_db[:, RANGE_FFT_LEN//2:]

            # --- Update Plot ---
            img_range_doppler.set_data(range_doppler_to_plot_db)
            
            current_max_db = np.max(range_doppler_to_plot_db)
            img_range_doppler.set_clim(vmin=current_max_db - 40, vmax=current_max_db) # 40dB dynamic range
            
            ax.set_title(f"Range-Doppler Spectrum (Frame: {frame_num}, Ant: {ANTENNA_INDEX_TO_DISPLAY+1})")
            
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        
        plt.pause(0.01) # Small pause to allow GUI to update and not run too fast

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
    plt.ioff()
    print("--- Script Finished ---")
    if plt.fignum_exists(fig.number):
         print("Close the plot window to exit completely.")
         plt.show(block=True)
