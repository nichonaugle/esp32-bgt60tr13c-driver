import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import time
import re
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from collections import deque
import json
import os
# New dependency for clustering
from scipy.ndimage import label

# --- Radar Configuration ---
NUM_CHIRPS_PER_FRAME = 64
NUM_SAMPLES_PER_CHIRP = 128
NUM_RX_ANTENNAS = 3
EXPECTED_SAMPLES_IN_FLAT_ARRAY = NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS

# Serial config
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'
BAUD_RATE = 921600

class SimpleCalibration:
    def __init__(self):
        self.range_baseline = None
        self.doppler_baseline = None
        self.is_calibrating = False

        # Sensitivity Controls
        self.range_sensitivity = 8.0   # dB above baseline for presence
        self.motion_sensitivity = 3.0  # Energy threshold (dB) for a pixel to be considered for motion
        self.min_cluster_size = 4      # Number of connected pixels to count as valid motion

        print("Simple calibration initialized")

    def start_calibration(self):
        self.is_calibrating = True
        print("Ready to calibrate EMPTY ROOM - will use next frame as NO-PRESENCE baseline...")

    def calibrate_with_current_frame(self, range_fft_db, range_doppler_db):
        if not self.is_calibrating:
            return False

        print("Calibrating EMPTY ROOM baseline with current frame...")
        range_roi = range_fft_db[:, 10:50]
        self.range_baseline = np.mean(range_roi)

        center_doppler = range_doppler_db.shape[0] // 2
        doppler_roi = range_doppler_db[center_doppler-5:center_doppler+5, :]
        self.doppler_baseline = np.mean(doppler_roi)

        self.is_calibrating = False
        print(f"EMPTY ROOM baselines: Range={self.range_baseline:.1f} dB, Doppler={self.doppler_baseline:.1f} dB")
        return True

    def detect_presence(self, range_fft_db):
        """ REVERTED to the original, more stable mean-based detection. """
        if self.range_baseline is None:
            return False, 0.0

        roi = range_fft_db[:, 10:50]
        # Reverted to using mean, as max was too sensitive.
        current_energy = np.mean(roi)

        energy_increase = current_energy - self.range_baseline
        presence_detected = energy_increase > self.range_sensitivity

        return presence_detected, energy_increase

    def detect_motion(self, range_doppler_db):
        """ ⭐ NEW: Cluster-based motion detection. """
        if self.doppler_baseline is None:
            return 0.0, False

        # 1. Define ROI
        # Focus on range bins where we expect targets
        roi = range_doppler_db[:, 10:50]
        # Ignore the very center of the doppler axis (static objects)
        center_doppler = roi.shape[0] // 2
        roi[center_doppler-2:center_doppler+2, :] = -100 # Effectively ignore this static zone

        # 2. Thresholding
        # Create a binary mask where any pixel above the energy threshold is 1
        energy_threshold = self.doppler_baseline + self.motion_sensitivity
        motion_mask = roi > energy_threshold

        # 3. Find Clusters (Connected-Component Labeling)
        # 'labeled_mask' will have a unique integer for each separate cluster
        # 'num_features' is the number of clusters found
        labeled_mask, num_features = label(motion_mask)

        if num_features == 0:
            return 0.0, False # No clusters found

        # 4. Evaluate Clusters
        # Get the size of each cluster
        cluster_sizes = np.bincount(labeled_mask.ravel())[1:] # [1:] to ignore background

        # Get the peak energy of each cluster
        # This is more for future use, but it's good to have
        peak_energies = [np.max(roi[labeled_mask == i]) for i in range(1, num_features + 1)]
        
        # Calculate a motion score based on the largest cluster
        motion_level = np.max(cluster_sizes) if cluster_sizes.size > 0 else 0.0

        # 5. Make Decision
        # Check if any of the found clusters are larger than our minimum size
        for size in cluster_sizes:
            if size >= self.min_cluster_size:
                return motion_level, True # Valid motion detected

        return motion_level, False # No clusters were large enough

    def save_calibration(self, filename="simple_radar_calibration.json"):
        data = {
            'range_baseline': self.range_baseline,
            'doppler_baseline': self.doppler_baseline,
            'range_sensitivity': self.range_sensitivity,
            'motion_sensitivity': self.motion_sensitivity,
            'min_cluster_size': self.min_cluster_size
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration saved to {filename}")

    def load_calibration(self, filename="simple_radar_calibration.json"):
        if not os.path.exists(filename):
            return False
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            self.range_baseline = data.get('range_baseline')
            self.doppler_baseline = data.get('doppler_baseline')
            self.range_sensitivity = data.get('range_sensitivity', 8.0)
            self.motion_sensitivity = data.get('motion_sensitivity', 3.0)
            self.min_cluster_size = data.get('min_cluster_size', 4)
            print(f"Calibration loaded from {filename}")
            return True
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False

# Initialize calibration
calibration = SimpleCalibration()

# GUI for calibration controls
class CalibrationControl:
    def __init__(self):
        self.window = tk.Toplevel()
        self.window.title("Calibration Control")
        self.window.geometry("400x550") # Increased height for new control
        self.window.attributes('-topmost', True)

        self.presence_var = tk.StringVar(value="NO")
        self.motion_var = tk.StringVar(value="NO")
        self.motion_level_var = tk.StringVar(value="0.0")

        self.latest_presence = False
        self.latest_motion = False
        self.latest_motion_level = 0.0

        self.setup_gui()
        self.fast_update_gui()

    def setup_gui(self):
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)

        cal_frame = ttk.LabelFrame(main_frame, text="Calibration")
        cal_frame.pack(fill='x', pady=(0, 10))

        button_frame = ttk.Frame(cal_frame)
        button_frame.pack(fill='x', padx=5, pady=5)
        ttk.Button(button_frame, text="Start Calibration", command=self.start_calibration).pack(side='left', padx=(0, 5))
        ttk.Button(button_frame, text="Save", command=self.save_calibration).pack(side='left', padx=(0, 5))
        ttk.Button(button_frame, text="Load", command=self.load_calibration).pack(side='left')
        self.cal_status = ttk.Label(cal_frame, text="Ready to calibrate")
        self.cal_status.pack(pady=5)

        status_frame = ttk.LabelFrame(main_frame, text="Detection Status")
        status_frame.pack(fill='x', pady=(0, 10))
        ttk.Label(status_frame, text="Presence:").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.presence_var, font=('Arial', 10, 'bold')).grid(row=0, column=1, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, text="Motion:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.motion_var, font=('Arial', 10, 'bold')).grid(row=1, column=1, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, text="Largest Cluster:").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.motion_level_var).grid(row=2, column=1, sticky='w', padx=5, pady=2)

        sens_frame = ttk.LabelFrame(main_frame, text="Sensitivity")
        sens_frame.pack(fill='x', pady=(0, 10))

        ttk.Label(sens_frame, text="Presence Sensitivity (dB):").pack(anchor='w', padx=5, pady=(5, 0))
        self.range_var = tk.DoubleVar(value=calibration.range_sensitivity)
        range_spinbox = ttk.Spinbox(sens_frame, from_=1.0, to=30.0, increment=0.1, textvariable=self.range_var, command=self.update_sensitivities, width=8)
        range_spinbox.pack(anchor='w', padx=5, pady=2)
        self.range_var.trace_add('write', self.update_sensitivities)

        ttk.Label(sens_frame, text="Motion Energy Threshold (dB):").pack(anchor='w', padx=5, pady=(10, 0))
        self.motion_sens_var = tk.DoubleVar(value=calibration.motion_sensitivity)
        motion_spinbox = ttk.Spinbox(sens_frame, from_=-10.0, to=20.0, increment=0.1, textvariable=self.motion_sens_var, command=self.update_sensitivities, width=8)
        motion_spinbox.pack(anchor='w', padx=5, pady=2)
        self.motion_sens_var.trace_add('write', self.update_sensitivities)

        # ⭐ NEW CONTROL for cluster size
        ttk.Label(sens_frame, text="Min Cluster Size (pixels):").pack(anchor='w', padx=5, pady=(10, 0))
        self.cluster_size_var = tk.IntVar(value=calibration.min_cluster_size)
        cluster_spinbox = ttk.Spinbox(sens_frame, from_=1, to=100, increment=1, textvariable=self.cluster_size_var, command=self.update_sensitivities, width=8)
        cluster_spinbox.pack(anchor='w', padx=5, pady=2)
        self.cluster_size_var.trace_add('write', self.update_sensitivities)


        info_frame = ttk.LabelFrame(main_frame, text="Info")
        info_frame.pack(fill='both', expand=True)
        self.info_text = tk.Text(info_frame, height=8, wrap=tk.WORD)
        self.info_text.pack(fill='both', expand=True, padx=5, pady=5)
        self.update_info()

    def update_sensitivities(self, *args):
        try:
            calibration.range_sensitivity = self.range_var.get()
            calibration.motion_sensitivity = self.motion_sens_var.get()
            calibration.min_cluster_size = self.cluster_size_var.get()
            self.update_info()
        except (tk.TclError, ValueError):
            pass

    def start_calibration(self):
        calibration.start_calibration()
        self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")

    def save_calibration(self):
        calibration.save_calibration()
        messagebox.showinfo("Save", "Calibration saved!")

    def load_calibration(self):
        if calibration.load_calibration():
            self.range_var.set(calibration.range_sensitivity)
            self.motion_sens_var.set(calibration.motion_sensitivity)
            self.cluster_size_var.set(calibration.min_cluster_size)
            self.update_info()
            messagebox.showinfo("Load", "Calibration loaded!")
        else:
            messagebox.showerror("Load", "Failed to load!")

    def update_status(self, presence, motion, motion_level):
        self.latest_presence = presence
        self.latest_motion = motion
        self.latest_motion_level = motion_level

    def fast_update_gui(self):
        self.presence_var.set("YES" if self.latest_presence else "NO")
        self.motion_var.set("YES" if self.latest_motion else "NO")
        self.motion_level_var.set(f"{self.latest_motion_level:.0f}")

        if calibration.is_calibrating:
            self.cal_status.config(text="Ready to calibrate EMPTY ROOM...")
        elif calibration.range_baseline is not None:
            self.cal_status.config(text="✓ Calibrated (Empty Room)")
        else:
            self.cal_status.config(text="Not calibrated")
        self.window.after(100, self.fast_update_gui)

    def update_info(self):
        info = "Calibration Status:\n\n"
        if calibration.range_baseline is not None:
            info += f"✓ Empty room range baseline: {calibration.range_baseline:.1f} dB\n"
            info += f"✓ Empty room doppler baseline: {calibration.doppler_baseline:.1f} dB\n\n"
            info += "Settings:\n"
            info += f"• Presence sensitivity: {calibration.range_sensitivity:.1f} dB\n"
            info += f"• Motion energy threshold: {calibration.motion_sensitivity:.1f} dB\n"
            info += f"• Min cluster size: {calibration.min_cluster_size} pixels\n\n"
            info += "Detection logic:\n"
            info += "• Presence = AVG range energy > baseline\n"
            info += "• Motion = Find clusters of pixels > energy threshold that are > min cluster size"
        else:
            info += "✗ Not calibrated\n\n"
            info += "Instructions:\n1. LEAVE THE ROOM\n2. Click 'Start Calibration'\n3. Come back and adjust sensitivity\n4. Save calibration"

        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(1.0, info)


# Main script logic (mostly unchanged from here)

def start_gui():
    control = CalibrationControl()
    return control

control_gui = start_gui()

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    exit()

rx_frame_channels = [np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32) for _ in range(NUM_RX_ANTENNAS)]
ANTENNA_INDEX_TO_DISPLAY = 0

plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(12, 20))

# Plot 1: Raw data
img_raw = axs[0].imshow(np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)), cmap='viridis', aspect='auto', extent=[0, NUM_SAMPLES_PER_CHIRP, 0, NUM_CHIRPS_PER_FRAME])
cb_raw = plt.colorbar(img_raw, ax=axs[0], label='Magnitude')
axs[0].set_xlabel('Sample Index'); axs[0].set_ylabel('Chirp Index'); axs[0].set_title(f'Raw Data (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 2: Range-FFT
img_range_fft = axs[1].imshow(np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)), cmap='viridis', aspect='auto', extent=[-NUM_SAMPLES_PER_CHIRP // 2, NUM_SAMPLES_PER_CHIRP // 2, 0, NUM_CHIRPS_PER_FRAME])
cb_range_fft = plt.colorbar(img_range_fft, ax=axs[1], label='Magnitude (dB)')
axs[1].set_xlabel('Range Bin (Shifted)'); axs[1].set_ylabel('Chirp Index'); axs[1].set_title(f'Range-FFT (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 3: Range-doppler
img_doppler_fft = axs[2].imshow(np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)), cmap='viridis', aspect='auto', extent=[0, NUM_SAMPLES_PER_CHIRP, -NUM_CHIRPS_PER_FRAME // 2, NUM_CHIRPS_PER_FRAME // 2])
cb_doppler_fft = plt.colorbar(img_doppler_fft, ax=axs[2], label='Magnitude (dB)')
axs[2].set_xlabel('Range Bin'); axs[2].set_ylabel('Doppler Bin'); axs[2].set_title(f'Range-Doppler (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 4: Detection status
axs[3].text(0.1, 0.8, 'Detection Status', fontsize=16, weight='bold'); axs[3].text(0.1, 0.6, 'Use Control Window', fontsize=14)
axs[3].set_xlim(0, 1); axs[3].set_ylim(0, 1); axs[3].axis('off')

fig.tight_layout(pad=3.0)

range_window = np.hanning(NUM_SAMPLES_PER_CHIRP)
doppler_window = np.hanning(NUM_CHIRPS_PER_FRAME)

def parse_frame_data_from_serial():
    try:
        while plt.fignum_exists(fig.number):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return None, None, False
            match = re.match(r"Frame (\d+): \[(.*)\]", line)
            if match:
                frame_num_str, data_str = match.groups()
                try:
                    flat_data_1d = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                except ValueError: continue
                if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    idx = 0
                    for c in range(NUM_CHIRPS_PER_FRAME):
                        for s in range(NUM_SAMPLES_PER_CHIRP):
                            for r_ant in range(NUM_RX_ANTENNAS):
                                rx_frame_channels[r_ant][c, s] = flat_data_1d[idx]
                                idx += 1
                    return rx_frame_channels[ANTENNA_INDEX_TO_DISPLAY], int(frame_num_str), False
                else: continue
        return None, None, True
    except Exception as e:
        print(f"An error occurred in parse_frame_data_from_serial: {e}")
        return None, None, False

print("Starting radar with cluster-based motion detection...")
print("Use the Control Window to calibrate and adjust sensitivity")

try:
    while True:
        if not plt.fignum_exists(fig.number):
            print("Plot window closed. Exiting."); break
        selected_antenna_frame_data, parsed_frame_num, should_exit = parse_frame_data_from_serial()
        if should_exit: break
        if selected_antenna_frame_data is None:
            plt.pause(0.05); continue

        # Standard processing pipeline...
        mean_per_chirp = np.mean(selected_antenna_frame_data, axis=1, keepdims=True)
        data_after_chirp_dc_removal = selected_antenna_frame_data - mean_per_chirp
        raw_data_to_plot = np.abs(data_after_chirp_dc_removal)
        img_raw.set_data(raw_data_to_plot)
        raw_min, raw_max = np.percentile(raw_data_to_plot, [1, 99]); img_raw.set_clim(vmin=raw_min, vmax=raw_max)
        axs[0].set_title(f'Raw Data (Frame: {parsed_frame_num})')

        windowed_for_range = data_after_chirp_dc_removal * range_window[np.newaxis, :]
        range_fft_output = np.fft.fft(windowed_for_range, axis=1)
        range_fft_shifted_for_plot = np.fft.fftshift(range_fft_output, axes=1)
        range_fft_db = 20 * np.log10(np.abs(range_fft_shifted_for_plot) + 1e-9)
        img_range_fft.set_data(range_fft_db)
        rfft_min, rfft_max = np.percentile(range_fft_db, [5, 99]); img_range_fft.set_clim(vmin=rfft_min, vmax=rfft_max)
        axs[1].set_title(f'Range-FFT (Frame: {parsed_frame_num})')

        mean_per_range_bin = np.mean(range_fft_output, axis=0, keepdims=True)
        range_fft_dc_removed_for_doppler = range_fft_output - mean_per_range_bin
        windowed_for_doppler = range_fft_dc_removed_for_doppler * doppler_window[:, np.newaxis]
        doppler_fft_output = np.fft.fft(windowed_for_doppler, axis=0)
        doppler_fft_shifted_for_plot = np.fft.fftshift(doppler_fft_output, axes=0)
        spectrogram_db = 20 * np.log10(np.abs(doppler_fft_shifted_for_plot) + 1e-9)
        img_doppler_fft.set_data(spectrogram_db)
        s_min, s_max = np.percentile(spectrogram_db, [5, 99]); img_doppler_fft.set_clim(vmin=s_min, vmax=s_max)
        axs[2].set_title(f'Range-Doppler (Frame: {parsed_frame_num})')

        if calibration.is_calibrating:
            if calibration.calibrate_with_current_frame(range_fft_db, spectrogram_db):
                control_gui.update_info()

        presence, presence_score = calibration.detect_presence(range_fft_db)
        motion_level, motion = calibration.detect_motion(spectrogram_db)
        control_gui.update_status(presence, motion, motion_level)

        axs[3].clear()
        axs[3].text(0.1, 0.9, 'Cluster-Based Detection', fontsize=16, weight='bold')
        axs[3].text(0.1, 0.7, f"Presence: {'YES' if presence else 'NO'}", fontsize=12, color='green' if presence else 'red')
        axs[3].text(0.1, 0.6, f"Motion: {'YES' if motion else 'NO'}", fontsize=12, color='orange' if motion else 'blue')
        axs[3].text(0.1, 0.5, f"Largest Cluster Size: {motion_level:.0f} pixels", fontsize=12)
        
        status = "LIGHTS_ON" if presence and motion else "KEEP_CURRENT" if presence and not motion else "LIGHTS_OFF"
        axs[3].text(0.1, 0.4, f"Status: {status}", fontsize=12, weight='bold', color='green' if status=='LIGHTS_ON' else 'red' if status=='LIGHTS_OFF' else 'blue')
        
        cal_status = "Calibrating..." if calibration.is_calibrating else "Calibrated" if calibration.range_baseline is not None else "Not Calibrated"
        axs[3].text(0.1, 0.3, f"Calibration: {cal_status}", fontsize=12)
        axs[3].set_xlim(0, 1); axs[3].set_ylim(0, 1); axs[3].axis('off')

        plt.draw()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Program stopped (Ctrl+C).")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    plt.ioff()
    if plt.fignum_exists(fig.number):
        print("Close plot window to exit.")
        plt.show(block=True)
    print("Program done.")