import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import time
import re

# --- Radar Configuration ---
# !!! IMPORTANT: NEEDS to match the bgt60tr13c_config.h on the ESP32 !!!
NUM_CHIRPS_PER_FRAME = 32
NUM_SAMPLES_PER_CHIRP = 64
NUM_RX_ANTENNAS = 3
# Expected samples = chirps * samples_per_chirp * antennas
EXPECTED_SAMPLES_IN_FLAT_ARRAY = NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS

# Serial config
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'
BAUD_RATE = 921600 # MAKE SURE THIS MATCHES THE ESP32 MONITOR BAUD RATE!!!!!!!!!!!!!!!!!!!!!!!!!!! OR IT WONT WORK!!!! BRUHHH!!!!!!!!!!!!!!!

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    print("Make sure the correct SERIAL_PORT is set in the script, the ESP32 is connected, and a different program isn't using the port.")
    exit()

# Initialize plot arrays for de-interleaving
rx_frame_channels = [
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32),
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32),
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32)
]
ANTENNA_INDEX_TO_DISPLAY = 0 # RX3 (0 for RX1, 1 for RX2, 2 for RX3)

# Initialize the plots
plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(12, 18))

# Plot 1: Raw data after per-chirp DC removal (tries to get rid of that annoying vertical line)
raw_data_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_raw = axs[0].imshow(np.zeros(raw_data_shape), cmap='viridis', aspect='auto',
                        extent=[0, NUM_SAMPLES_PER_CHIRP, 0, NUM_CHIRPS_PER_FRAME])
cb_raw = plt.colorbar(img_raw, ax=axs[0], label='Magnitude')
axs[0].set_xlabel('Sample Index (Samples per Chirp)')
axs[0].set_ylabel('Chirp Index')
axs[0].set_title(f'Raw Data (Post Per-Chirp DC Removal) (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 2: Range-FFT Output in dB
range_fft_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_range_fft = axs[1].imshow(np.zeros(range_fft_shape), cmap='viridis', aspect='auto',
                              extent=[-NUM_SAMPLES_PER_CHIRP // 2, NUM_SAMPLES_PER_CHIRP // 2, 0, NUM_CHIRPS_PER_FRAME])
cb_range_fft = plt.colorbar(img_range_fft, ax=axs[1], label='Magnitude (dB)')
axs[1].set_xlabel('Range Bin (Shifted)')
axs[1].set_ylabel('Chirp Index')
axs[1].set_title(f'Range-FFT Output (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 3: Final range-doppler spectrogram in dB
doppler_fft_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_doppler_fft = axs[2].imshow(np.zeros(doppler_fft_shape), cmap='viridis', aspect='auto',
                                extent=[0, NUM_SAMPLES_PER_CHIRP, -NUM_CHIRPS_PER_FRAME // 2, NUM_CHIRPS_PER_FRAME // 2])
cb_doppler_fft = plt.colorbar(img_doppler_fft, ax=axs[2], label='Magnitude (dB)')
axs[2].set_xlabel('Range Bin (Samples per Chirp)')
axs[2].set_ylabel('Doppler Bin (Chirps per Frame)')
axs[2].set_title(f'Range-Doppler Spectrogram (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

fig.tight_layout(pad=3.0)

# Create windowing functions
range_window = np.hanning(NUM_SAMPLES_PER_CHIRP)
doppler_window = np.hanning(NUM_CHIRPS_PER_FRAME)

def parse_frame_data_from_serial():
    try:
        while True:
            if not plt.fignum_exists(fig.number):
                return None, None, True

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None, None, False

            match = re.match(r"Frame (\d+): \[(.*)\]", line)
            if match:
                frame_num_str, data_str = match.groups()
                frame_num_esp = int(frame_num_str)

                try:
                    flat_data_1d = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                except ValueError as e:
                    print(f"Error when converting data string for frame {frame_num_esp}: {e}")
                    continue

                if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    idx = 0
                    for c in range(NUM_CHIRPS_PER_FRAME):
                        for s in range(NUM_SAMPLES_PER_CHIRP):
                            for r_ant in range(NUM_RX_ANTENNAS):
                                if idx < len(flat_data_1d):
                                    rx_frame_channels[r_ant][c, s] = flat_data_1d[idx]
                                    idx += 1
                                else:
                                    print("Oops, ran out of data while de-interleaving.")
                                    return None, None, False
                    return rx_frame_channels[ANTENNA_INDEX_TO_DISPLAY], frame_num_esp, False
                else:
                    print(f"Frame {frame_num_esp}: Data length mismatch. Expected {EXPECTED_SAMPLES_IN_FLAT_ARRAY}, got {len(flat_data_1d)}.")
                    print(f"    Expected: {NUM_CHIRPS_PER_FRAME} chirps, {NUM_SAMPLES_PER_CHIRP} samples/chirp.")
                    print(f"    Check bgt60tr13c_config.h on ESP32 and make sure this script matches.")
                    continue
    except serial.SerialTimeoutException:
        # can happen normally if no data is sent within the timeout period
        return None, None, False
    except Exception as e:
        print(f"An error occurred in parse_frame_data_from_serial: {e}")
        if ser and ser.is_open:
            ser.reset_input_buffer()
        return None, None, False

print("Starting the data acquisition and plotting loop...")
print(f"IMPORTANT!!!: Python script configured for {NUM_CHIRPS_PER_FRAME} chirps/frame, {NUM_SAMPLES_PER_CHIRP} samples/chirp. Make sure it matches ESP32!!!")
print(f"Displaying RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1}")
print(f"EXPECTED_SAMPLES_IN_FLAT_ARRAY per frame (total for all RX): {EXPECTED_SAMPLES_IN_FLAT_ARRAY}")


try:
    while True:
        if not plt.fignum_exists(fig.number):
            print("Plot window closed. Exiting.")
            break

        selected_antenna_frame_data, parsed_frame_num, should_exit = parse_frame_data_from_serial()

        if should_exit:
            break

        if selected_antenna_frame_data is None:
            plt.pause(0.05) 
            continue

        if selected_antenna_frame_data.shape != (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP):
            print(f"Error: Parsed frame data has unexpected shape {selected_antenna_frame_data.shape}. Expected ({NUM_CHIRPS_PER_FRAME}, {NUM_SAMPLES_PER_CHIRP}). So skipping it.")
            continue

        # --- Per-chirp DC offset removal ---
        mean_per_chirp = np.mean(selected_antenna_frame_data, axis=1, keepdims=True)
        data_after_chirp_dc_removal = selected_antenna_frame_data - mean_per_chirp

        # --- Plot 1: Raw Data (Now shows data after per-chirp DC removal) ---
        raw_data_to_plot = np.abs(data_after_chirp_dc_removal)
        img_raw.set_data(raw_data_to_plot)
        raw_min = np.percentile(raw_data_to_plot.flatten(), 1)
        raw_max = np.percentile(raw_data_to_plot.flatten(), 99)
        if raw_min >= raw_max : raw_max = raw_min + 1e-3 # Make sure max is bigger than min
        img_raw.set_clim(vmin=raw_min, vmax=raw_max)
        axs[0].set_title(f'Raw Data (Post Per-Chirp DC Removal) (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1}, Frame: {parsed_frame_num if parsed_frame_num is not None else "N/A"})')

        # --- Signal processing for Plot 2 (Range-FFT) & Plot 3 (Doppler-FFT) ---
        # Apply range windowing (Use DC-removed data)
        windowed_for_range = data_after_chirp_dc_removal * range_window[np.newaxis, :]

        # Range-FFT (along the samples axis)
        range_fft_output = np.fft.fft(windowed_for_range, axis=1)

        # --- Plot 2: Range-FFT output ---
        range_fft_shifted_for_plot = np.fft.fftshift(range_fft_output, axes=1)
        range_fft_db = 20 * np.log10(np.abs(range_fft_shifted_for_plot) + 1e-9) # Add epsilon for log(0)
        img_range_fft.set_data(range_fft_db)
        rfft_min = np.percentile(range_fft_db.flatten(), 5)
        rfft_max = np.percentile(range_fft_db.flatten(), 99)
        if rfft_min >= rfft_max : rfft_max = rfft_min + 1 # Make sure max is bigger than min
        img_range_fft.set_clim(vmin=rfft_min, vmax=rfft_max)
        axs[1].set_title(f'Range-FFT (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1}, Frame: {parsed_frame_num if parsed_frame_num is not None else "N/A"})')

        # --- Continue the processing for Plot 3 (Final Spectrogram) ---
        # DC offset removal for doppler
        #    this uses the output of the Range FFT (range_fft_output)
        mean_per_range_bin = np.mean(range_fft_output, axis=0, keepdims=True)
        range_fft_dc_removed_for_doppler = range_fft_output - mean_per_range_bin

        # Doppler windowing
        windowed_for_doppler = range_fft_dc_removed_for_doppler * doppler_window[:, np.newaxis]

        # Doppler-FFT (along chirps axis)
        doppler_fft_output = np.fft.fft(windowed_for_doppler, axis=0)

        # Shift zero-frequency component to center for doppler visualization
        doppler_fft_shifted_for_plot = np.fft.fftshift(doppler_fft_output, axes=0)

        # Calculate magnitude and convert to dB
        spectrogram_db = 20 * np.log10(np.abs(doppler_fft_shifted_for_plot) + 1e-9) # Add epsilon

        # --- Plot 3: Final Range-doppler dpectrogram ---
        img_doppler_fft.set_data(spectrogram_db)
        s_min = np.percentile(spectrogram_db.flatten(), 5)
        s_max = np.percentile(spectrogram_db.flatten(), 99)
        if s_min >= s_max : s_max = s_min + 1 # make sure max is bigger than min
        img_doppler_fft.set_clim(vmin=s_min, vmax=s_max)
        axs[2].set_title(f'Range-Doppler (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1}, Frame: {parsed_frame_num if parsed_frame_num is not None else "N/A"})')

        plt.draw()
        plt.pause(0.01) # processing takes time

except KeyboardInterrupt:
    print("Program stopped (Ctrl+C).")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    plt.ioff()
    if plt.fignum_exists(fig.number): # check if the figure actually still exists before trying to show it
       print("Displaying final plots. Close the plot window to exit.")
       plt.show(block=True) # keep it open until manually closed
    print("Program done.")