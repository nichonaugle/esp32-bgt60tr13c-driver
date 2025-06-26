import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fft2, fftshift
from scipy.ndimage import median_filter
from scipy.constants import c
from matplotlib.gridspec import GridSpec

# Radar parameters (adjust these)
# CONSTANTS
f_low = 60479900000
f_high = 61479900000
f_bandwidth = abs(f_high-f_low)
Tc = 0.00005738 # chirp time
shape_end_delay = 0.000690
sample_rate = 2352941
PRT = shape_end_delay + Tc
samples_per_chirp = 128
chirps_per_frame = 64

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

fft_len = samples_per_chirp * 4
ranges = np.linspace(-r_max, r_max, fft_len)

ser = serial.Serial('/dev/ttyUSB0', 921600)  # Change COMX to your port

# Setup main plot
plt.ion()
fig = plt.figure(figsize=(14, 12)) # Create the figure first

# Define a GridSpec object: 3 rows, 2 columns
gs = GridSpec(3, 2, figure=fig)

# Assign subplots using GridSpec slicing
ax_main = fig.add_subplot(gs[:, 0]) # Spans all rows (:) in the first column (0)
ax_beat = fig.add_subplot(gs[0, 1]) # First row (0), second column (1)
ax_fft = fig.add_subplot(gs[1, 1])  # Second row (1), second column (1)
ax_raw_data = fig.add_subplot(gs[2, 1]) # Third row (2), second column (1)

#fig, ((ax_main), (ax_beat, ax_fft, ax_raw_data)) = plt.subplots(1, 2, figsize=(10, 12))

# Range-Doppler plot
im = ax_main.imshow(np.zeros((samples_per_chirp, chirps_per_frame)),
                   aspect='auto',
                   extent=[-v_max/2, v_max/2, ranges.min(), ranges.max()],
                   origin='lower',
                   vmin=-50,
                   vmax=30)
fig.colorbar(im, ax=ax_main)
ax_main.set_title("Range Doppler Spectrum")
ax_main.set_xlabel("Velocity (m/s)")
ax_main.set_ylabel("Range (m)")

# Single FFT plot for chirps beat signal
ax_beat.set_title("All Chirps beat Signal")
ax_beat.set_xlabel("Sample #")
ax_beat.set_ylabel("Amplitude")
ax_beat.set_xlim([0, samples_per_chirp])
ax_beat.set_ylim([-1, 1])

# Single FFT plot for all chirps
ax_fft.set_title("All Chirps Frequency Spectrum")
ax_fft.set_xlabel("Range (m)")
ax_fft.set_ylabel("Magnitude (dB)")
ax_fft.set_xlim([0, r_max])
ax_fft.set_ylim([-30, 20])

# NEW: Raw Radar Data 2D Frame Plot
im_raw = ax_raw_data.imshow(np.zeros((chirps_per_frame, samples_per_chirp)),
                           aspect='auto',
                           cmap='viridis',
                           origin='lower',
                           extent=[0, samples_per_chirp, 0, chirps_per_frame])
fig.colorbar(im_raw, ax=ax_raw_data)
ax_raw_data.set_title("Raw Radar Data (Chirp vs Sample)")
ax_raw_data.set_xlabel("Sample Index")
ax_raw_data.set_ylabel("Chirp Index")

# Colormap for different chirps
colors = plt.cm.viridis(np.linspace(0, 1, chirps_per_frame))

num_frames_to_average = 1
accumulated_rd_map = np.zeros((samples_per_chirp, chirps_per_frame))
frame_count = 0

while True:
    try:
        line = ser.readline().decode().strip()
        if not line.startswith('Frame'): 
            print(line)
            continue
        
        # Safely extract data
        data_str = line.split(':', 1)[1].strip()
        data = eval(data_str) if '[' in data_str else None
        if not data or len(data) < chirps_per_frame * samples_per_chirp: continue
        
        # Process frame (USE ONLY 1 ANTENNA)
        radar_data = np.array([float(x) for x in data], dtype=np.float32)
        if radar_data.shape[0] < samples_per_chirp * chirps_per_frame:
            padding_needed = samples_per_chirp * chirps_per_frame - radar_data.shape[0]
            radar_data = np.pad(radar_data, (0, padding_needed), 'constant', constant_values=0)
            print(f"Radar padding Added")
        print(f"Radar dataset size: {radar_data.shape[0]}\n")
        radar_data = radar_data.reshape((chirps_per_frame, samples_per_chirp))
        print(f"Radar dataset shape: {radar_data.shape}\n")
        
        # NEW: Update raw data plot
        im_raw.set_data(radar_data)
        im_raw.set_clim(vmin=np.min(radar_data), vmax=np.max(radar_data)) # Adjust if you want fixed range

        ###########################################
        # Beat Signal processing
        ax_beat.clear()
        for i in range(chirps_per_frame):
            demodulated_signal = radar_data[i] - np.mean(radar_data[i])
            normalized_signal = (demodulated_signal / np.max(np.abs(demodulated_signal)))
            radar_data[i]=np.hanning(samples_per_chirp) * normalized_signal
            #if i%32 == 0:
            ax_beat.plot(radar_data[i], color=colors[i], alpha=0.8, label='Coherent Integration')

        ###########################################
        # Coherent integration across chirps
        # Sum FFTs of all chirps before taking magnitude (preserving phase)
        ax_fft.clear()
        for i in range(chirps_per_frame):
            X_k = fftshift(fft(radar_data[i], fft_len))
            X_k /= (samples_per_chirp / 2)
            X_k = 20 * np.log10(np.abs(X_k))
            if i%32 == 0:
                ax_fft.plot(ranges, X_k, color=colors[i], alpha=0.8, label='Coherent Integration')

        ax_fft.set_xlim([0, r_max])
        ax_fft.set_ylim([-80, 0])

        ###########################################
        # Range-Doppler processing
        raw_rd_map = fft2(radar_data.T)
        clutter_suppressed_rd_map = np.copy(raw_rd_map)
        clutter_suppressed_rd_map[:, 0] = raw_rd_map[:, 0] * 1#0.01 
        rd = fftshift(np.abs(clutter_suppressed_rd_map)) / (samples_per_chirp / 2)
        
        # Update plot data
        #rd_log = 10 * np.log10(rd)
        #im.set_data(rd_log)
        #im.set_clim(vmin=np.min(rd_log), vmax=np.max(rd_log))  # Dynamic scaling
        #ax_main.set_ylim([0, r_max])
        #fig.canvas.draw_idle()
        
        # Frame Averaging
        accumulated_rd_map += rd
        frame_count += 1

        if frame_count >= num_frames_to_average:
            averaged_rd_map = accumulated_rd_map / num_frames_to_average
            rd_log = 20 * np.log10(averaged_rd_map + 1e-10)

            #rd_log = median_filter(rd_log, size=3)

            im.set_data(rd_log)
            ax_main.set_ylim([0, r_max])
            fig.canvas.draw_idle()
            
            accumulated_rd_map.fill(0)
            frame_count = 0

        ###########################################
        # Render all plots
        fig.tight_layout()
        fig.canvas.draw_idle()
        plt.pause(0.01)
        
    except Exception as e:
        print(f"Skipping frame due to: {str(e)}")
        continue