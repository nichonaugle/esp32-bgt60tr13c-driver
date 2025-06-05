import re
import numpy as np
from scipy.signal import windows
from scipy import constants
import matplotlib.pyplot as plt
import serial
import time

# --- Configuration ---
NUM_CHIRPS_PER_FRAME = 16
NUM_SAMPLES_PER_CHIRP = 256
NUM_RX_ANTENNAS = 1
BAUD_RATE = 921900
COM_PORT = '/dev/ttyUSB0'

START_FREQUENCY_HZ = 60e9
END_FREQUENCY_HZ = 62e9
BANDWIDTH_HZ = abs(END_FREQUENCY_HZ - START_FREQUENCY_HZ)
CHIRP_DURATION_S = 0.0001935
ADC_SAMPLE_RATE_SPS = 2e6

# --- Plotting Setup ---
fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(12, 18))

line0, = ax0.plot(np.arange((NUM_CHIRPS_PER_FRAME) *NUM_SAMPLES_PER_CHIRP), np.zeros((NUM_CHIRPS_PER_FRAME) *NUM_SAMPLES_PER_CHIRP))
ax0.set_title("Raw Frame Data from Serial")
ax0.set_xlabel("Sample Index")
ax0.set_ylabel("Amplitude")
ax0.set_ylim([0, 4096])
ax0.grid(True)

range_resolution_m = constants.c / (2 * BANDWIDTH_HZ)
range_bin_length_m = (constants.c) / (2 * BANDWIDTH_HZ * (NUM_SAMPLES_PER_CHIRP * 2) / NUM_SAMPLES_PER_CHIRP)
max_range_m = ((NUM_SAMPLES_PER_CHIRP * 2) // 2) * range_bin_length_m # Max range for positive frequencies
range_bins_m = np.linspace(0, max_range_m, (NUM_SAMPLES_PER_CHIRP * 2) // 2)

img_chirp_range = ax1.imshow(np.zeros((NUM_CHIRPS_PER_FRAME, (NUM_SAMPLES_PER_CHIRP * 2) // 2)), aspect='auto', origin='lower',
                             cmap='inferno', extent=[range_bins_m.min(), range_bins_m.max(),
                                                     0, NUM_CHIRPS_PER_FRAME - 1]) # Y-axis from 0 to Chirp-1
ax1.set_title("Chirp-Range Map (Range FFT per Chirp)")
ax1.set_xlabel("Range (m)")
ax1.set_ylabel("Chirp Index")
plt.colorbar(img_chirp_range, ax=ax1, label='Magnitude (dB)')
ax1.grid(False)

# --- ax2: Range-Doppler Map Plot ---
doppler_resolution_hz = 1 / ((NUM_CHIRPS_PER_FRAME) * CHIRP_DURATION_S)
max_doppler_hz = NUM_CHIRPS_PER_FRAME / 2 * doppler_resolution_hz
doppler_freqs_hz = np.linspace(-max_doppler_hz, max_doppler_hz, NUM_CHIRPS_PER_FRAME)
carrier_frequency_hz = (START_FREQUENCY_HZ + END_FREQUENCY_HZ) / 2 # Using center frequency for lambda
max_velocity_mps = max_doppler_hz * (constants.c / (2 * carrier_frequency_hz))
velocity_bins_mps = np.linspace(-max_velocity_mps, max_velocity_mps, NUM_CHIRPS_PER_FRAME)

# Initialize the imshow plot. The data will be updated later.
# The extent argument defines the range of the x and y axes for the image.
img_display = ax2.imshow(np.zeros((NUM_CHIRPS_PER_FRAME, (NUM_CHIRPS_PER_FRAME * 2) // 2)), aspect='auto', origin='lower',
                        cmap='viridis', extent=[range_bins_m.min(), range_bins_m.max(),
                                                velocity_bins_mps.min(), velocity_bins_mps.max()])
ax2.set_title("Range-Doppler Map")
ax2.set_xlabel("Range (m)")
ax2.set_ylabel("Velocity (m/s)")
plt.colorbar(img_display, ax=ax2, label='Magnitude (dB)')
ax2.grid(False) # Grid usually not needed for imshow

plt.tight_layout()
plt.ion() # Turn on interactive mode
plt.show(block=False) # Show the plot without blocking

fig.canvas.draw()
fig.canvas.flush_events()
time.sleep(0.1)

def fft_spectrum(data_array, window_array, n_fft=None, axis=-1):
    """
    Applies a window to data and performs FFT along the specified axis.
    Args:
        data_array (np.ndarray): The input data.
        window_array (np.ndarray): The window to apply. Should be broadcastable to data_array.
        n_fft (int, optional): Number of FFT points. If None, uses data_array.shape[axis].
                                For zero-padding.
        axis (int, optional): The axis along which to compute the FFT. Defaults to -1.
    Returns:
        np.ndarray: The FFT output.
    """
    window_reshaped = window_array
    if window_array.ndim < data_array.ndim:
        target_shape_len = data_array.ndim - window_array.ndim
        window_reshaped = np.expand_dims(window_array, axis=tuple(range(target_shape_len)))

    windowed_data = data_array * window_reshaped
    return np.fft.fft(windowed_data, n=n_fft, axis=axis)

def render_radar_fft(data=None) -> None:
    # Fig 0: Raw Data
    line0.set_ydata(data)
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Fig 1: Distance Plot (Doppler)
    range_window = windows.blackmanharris(NUM_SAMPLES_PER_CHIRP).reshape(1, NUM_SAMPLES_PER_CHIRP)
    # Doppler window: (NUM_CHIRPS_PER_FRAME, 1) to broadcast across chirps dimension (axis=0)
    doppler_window = windows.hamming(NUM_CHIRPS_PER_FRAME).reshape(NUM_CHIRPS_PER_FRAME, 1)

    # Step 1: Perform Range FFT (along the samples dimension - axis 1)
    # Output shape: (NUM_CHIRPS_PER_FRAME, RANGE_FFT_SIZE) e.g., (16, 512) or (15, 512)
    range_fft_output = fft_spectrum(data, range_window, n_fft=(NUM_CHIRPS_PER_FRAME*2), axis=1)

    # --- Chirp-Range Map Plot ---
    chirp_range_magnitude = np.abs(range_fft_output)[:, :(NUM_CHIRPS_PER_FRAME*2) // 2]
    chirp_range_magnitude_db = 20 * np.log10(chirp_range_magnitude + 1e-10)

    img_chirp_range.set_array(chirp_range_magnitude_db)
    img_chirp_range.set_clim(np.min(chirp_range_magnitude_db), np.max(chirp_range_magnitude_db))


    # Step 2: Perform Doppler FFT (along the chirps dimension - axis 0)
    doppler_fft_output = fft_spectrum(range_fft_output, doppler_window, n_fft=NUM_CHIRPS_PER_FRAME, axis=0)

    # Step 3: Shift zero-frequency component to center for Doppler dimension (axis 0)
    range_doppler_map_shifted = np.fft.fftshift(doppler_fft_output, axes=0)
    plot_map_db = np.abs(range_doppler_map_shifted)[:, :(NUM_CHIRPS_PER_FRAME*2) // 2]
    range_doppler_plot_data = 10 * np.log10(plot_map_db / (NUM_CHIRPS_PER_FRAME / 2) + 1e-10) # Added +1e-10 for log stability

    img_display.set_array(range_doppler_plot_data)
    img_display.set_clim(np.min(range_doppler_plot_data), np.max(range_doppler_plot_data))
    # You can set fixed clim as per your reference:
    # img_display.set_clim(vmin=-25, vmax=2) # Uncomment to use fixed limits

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    

def run_radar_engine(serial_connection=None) -> None:
    time.sleep(0.05)
    serial_connection.flushInput()

    while True:
        line = serial_connection.readline().decode('utf-8', errors='ignore').strip()
        print(f"[DEBUG]: RECIVED LINE >>> {line}")
        match = re.match(r"Frame (\d+): \[(.*)\]", line)
        if match:
            _, data_str = match.groups()
            data = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
            data = data[::3].reshape((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP))
            render_radar_fft(data=data)


if __name__ == "__main__":
    try:
        ser = serial_connection=serial.Serial(COM_PORT, BAUD_RATE)
        run_radar_engine(
            serial_connection=ser
            )
    except KeyboardInterrupt:
        print("Loop interrupted by user.")
    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")
        print("Please check if the COM_PORT is correct and if the device is connected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        plt.close(fig)
        print("Plotting loop finished.")