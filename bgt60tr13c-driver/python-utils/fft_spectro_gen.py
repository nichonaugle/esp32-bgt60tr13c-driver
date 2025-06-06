import re
import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import matplotlib.pyplot as plt
from scipy.constants import c
from numpy.fft import fftshift, fft2

# --- Configuration ---
NUM_CHIRPS_PER_FRAME = 16
NUM_SAMPLES_PER_CHIRP = 256
NUM_RX_ANTENNAS = 3
BAUD_RATE = 921900
COM_PORT = '/dev/ttyUSB0'

# CONSTANTS
f_low = 58e9
f_high = 62e9
f_bandwidth = abs(f_high-f_low)
Tc = 0.0001335 # chirp time
shape_end_delay = 0.00006
sample_rate = 2e6
PRT = Tc + shape_end_delay
samples_per_chirp = 256
chirps_per_frame = 16

r_max = (sample_rate * c * Tc) / (4 * f_bandwidth)
print(f"Maximum range: {r_max:.3f} m")

# RADAR SPEC DEFINITIONS
r_res = c / (2 * f_bandwidth)
print(f"Range resolution: {r_res} m")

v_max =  c / (2 * f_bandwidth * PRT)
print(f"Maximum velocity: {v_max:.3f} m/s")

v_res = c / (2 * chirps_per_frame * PRT * f_low) # m/s
print(f"Velocity resolution: {v_res:.3f} m/s\n")

signal_capture_time =  chirps_per_frame * PRT
print(f"Signal capture time: {(signal_capture_time * 1e3)} ms")
print(f"Max FPS: {1/signal_capture_time} fps")

# --- Plotting Setup ---
fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(12, 18))

line0, = ax0.plot(np.arange((NUM_CHIRPS_PER_FRAME) *NUM_SAMPLES_PER_CHIRP), np.zeros((NUM_CHIRPS_PER_FRAME) *NUM_SAMPLES_PER_CHIRP))
ax0.set_title("Raw Frame Data from Serial")
ax0.set_xlabel("Sample Index")
ax0.set_ylabel("Amplitude")
ax0.set_ylim([0, 4096])
ax0.grid(True)

ax1.set_ylim([0, r_max])
ax1.set_title("Range Doppler Spectrum", fontsize=24)
ax1.set_xlabel("Velocity (m/s)", fontsize=22)
ax1.set_ylabel("Range (m)", fontsize=22)

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


    ranges = np.linspace(-r_max, r_max, samples_per_chirp * 4)

    for i in range(chirps_per_frame):
        demodulated_signal = data[i] - np.mean(data[i])
        normalized_signal = (demodulated_signal / np.max(np.abs(demodulated_signal)))
        data[i]=np.hanning(samples_per_chirp) * normalized_signal

    vels = np.linspace(-v_max, v_max, chirps_per_frame)

    range_doppler = fftshift(np.abs(fft2(data.T))) / (samples_per_chirp / 2)
    print(range_doppler.shape)
    extent = [-v_max/2, v_max/2, ranges.min(), ranges.max()]

    range_doppler_plot = ax1.imshow(
        10 * np.log10(range_doppler),
        aspect="auto",
        extent=extent,
        origin="lower",
        vmax=5,#np.max(10 * np.log10(range_doppler)),
        vmin=-25,#np.min(10 * np.log10(range_doppler)),
    )

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
            try:
                data = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                print(data.shape[0])
                if data.shape[0] == NUM_RX_ANTENNAS * NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP:
                    data = data[::3].reshape((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP))
                    render_radar_fft(data=data)
            except:
                print("Data Corruption, skipping frame...")
                continue


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