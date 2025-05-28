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
        # Simplified calibration - just overall baselines and sensitivities
        self.range_baseline = None
        self.doppler_baseline = None
        self.is_calibrating = False
        
        # Simple sensitivity controls
        self.range_sensitivity = 8.0   # dB above baseline for presence
        self.motion_sensitivity = 3.0  # dB for motion detection
        
        print("Simple calibration initialized")
    
    def start_calibration(self):
        self.is_calibrating = True
        print("Ready to calibrate EMPTY ROOM - will use next frame as NO-PRESENCE baseline...")
    
    def calibrate_with_current_frame(self, range_fft_db, range_doppler_db):
        """Immediate calibration with current frame data - should be EMPTY ROOM"""
        if not self.is_calibrating:
            return False
        
        print("Calibrating EMPTY ROOM baseline with current frame...")
        
        # Calculate baselines from EMPTY room frame
        # Range baseline - avoid DC bins (first few bins) - this should be LOW (no targets)
        range_roi = range_fft_db[:, 10:50]  # Skip DC, focus on target area
        self.range_baseline = np.mean(range_roi)
        
        # Doppler baseline - focus on center (static) region - should be LOW (no motion)
        center_doppler = range_doppler_db.shape[0] // 2
        doppler_roi = range_doppler_db[center_doppler-5:center_doppler+5, :]
        self.doppler_baseline = np.mean(doppler_roi)
        
        self.is_calibrating = False
        print(f"EMPTY ROOM baselines: Range={self.range_baseline:.1f} dB, Doppler={self.doppler_baseline:.1f} dB")
        print("Now presence/motion will be detected as INCREASES above these baselines")
        return True
    
    def detect_presence(self, range_fft_db):
        if self.range_baseline is None:
            return False, 0.0
        
        # Check range bins 10-50 (avoid DC, focus on target area)
        roi = range_fft_db[:, 10:50]
        current_energy = np.mean(roi)  # Current energy in target area
        
        # Presence = INCREASE above empty room baseline
        energy_increase = current_energy - self.range_baseline
        presence_detected = energy_increase > self.range_sensitivity
        
        return presence_detected, energy_increase
    
    def detect_motion(self, range_doppler_db):
        if self.doppler_baseline is None:
            return 0.0, False
        
        # Focus on range bins where we expect targets (10-50)
        roi = range_doppler_db[:, 10:50]
        
        center_doppler = roi.shape[0] // 2
        
        # Motion energy (edge bins) - should INCREASE from baseline when there's motion
        motion_roi_pos = roi[center_doppler+4:, :]
        motion_roi_neg = roi[:center_doppler-3, :]
        
        motion_energy = 0
        if motion_roi_pos.size > 0:
            motion_energy += np.mean(motion_roi_pos)
        if motion_roi_neg.size > 0:
            motion_energy += np.mean(motion_roi_neg)
        
        motion_energy = motion_energy / 2  # Average of positive and negative
        
        # Motion = INCREASE in edge doppler bins above empty room baseline
        motion_increase = motion_energy - self.doppler_baseline
        motion_detected = motion_increase > self.motion_sensitivity
        
        return motion_increase, motion_detected
    
    def save_calibration(self, filename="simple_radar_calibration.json"):
        data = {
            'range_baseline': self.range_baseline,
            'doppler_baseline': self.doppler_baseline,
            'range_sensitivity': self.range_sensitivity,
            'motion_sensitivity': self.motion_sensitivity
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration saved to {filename}")
    
    def load_calibration(self, filename="simple_radar_calibration.json"):
        if not os.path.exists(filename):
            print(f"Calibration file {filename} not found")
            return False
        
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.range_baseline = data.get('range_baseline')
            self.doppler_baseline = data.get('doppler_baseline')
            self.range_sensitivity = data.get('range_sensitivity', 8.0)
            self.motion_sensitivity = data.get('motion_sensitivity', 3.0)
            
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
        self.window.geometry("400x500")
        self.window.attributes('-topmost', True)  # Keep on top
        
        # Detection status variables (must be defined BEFORE setup_gui)
        self.presence_var = tk.StringVar(value="NO")
        self.motion_var = tk.StringVar(value="NO")
        self.motion_level_var = tk.StringVar(value="0.000")
        
        # Store latest detection results for fast updates
        self.latest_presence = False
        self.latest_motion = False
        self.latest_motion_level = 0.0
        
        self.setup_gui()
        
        # Start fast GUI update timer (independent of plots)
        self.fast_update_gui()
        
    def setup_gui(self):
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Calibration controls
        cal_frame = ttk.LabelFrame(main_frame, text="Calibration")
        cal_frame.pack(fill='x', pady=(0, 10))
        
        button_frame = ttk.Frame(cal_frame)
        button_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(button_frame, text="Start Calibration", 
                  command=self.start_calibration).pack(side='left', padx=(0, 5))
        ttk.Button(button_frame, text="Save", 
                  command=self.save_calibration).pack(side='left', padx=(0, 5))
        ttk.Button(button_frame, text="Load", 
                  command=self.load_calibration).pack(side='left')
        
        self.cal_status = ttk.Label(cal_frame, text="Ready to calibrate")
        self.cal_status.pack(pady=5)
        
        # Detection status
        status_frame = ttk.LabelFrame(main_frame, text="Detection Status")
        status_frame.pack(fill='x', pady=(0, 10))
        
        ttk.Label(status_frame, text="Presence:").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.presence_var, font=('Arial', 10, 'bold')).grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        ttk.Label(status_frame, text="Motion:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.motion_var, font=('Arial', 10, 'bold')).grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        ttk.Label(status_frame, text="Motion Level:").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.motion_level_var).grid(row=2, column=1, sticky='w', padx=5, pady=2)
        
        # Sensitivity controls
        sens_frame = ttk.LabelFrame(main_frame, text="Sensitivity")
        sens_frame.pack(fill='x', pady=(0, 10))
        
        # Range sensitivity
        ttk.Label(sens_frame, text="Presence Sensitivity:").pack(anchor='w', padx=5, pady=(5, 0))
        self.range_var = tk.DoubleVar(value=calibration.range_sensitivity)
        range_scale = ttk.Scale(sens_frame, from_=1, to=20, orient='horizontal',
                               variable=self.range_var, command=self.update_range_sensitivity)
        range_scale.pack(fill='x', padx=5, pady=2)
        self.range_label = ttk.Label(sens_frame, text=f"{calibration.range_sensitivity:.1f} dB")
        self.range_label.pack(anchor='w', padx=5)
        
        # Motion sensitivity
        ttk.Label(sens_frame, text="Motion Sensitivity:").pack(anchor='w', padx=5, pady=(10, 0))
        self.motion_var = tk.DoubleVar(value=calibration.motion_sensitivity)
        motion_scale = ttk.Scale(sens_frame, from_=0.5, to=10, orient='horizontal',
                                variable=self.motion_var, command=self.update_motion_sensitivity)
        motion_scale.pack(fill='x', padx=5, pady=2)
        self.motion_label = ttk.Label(sens_frame, text=f"{calibration.motion_sensitivity:.1f}")
        self.motion_label.pack(anchor='w', padx=5)
        
        # Info
        info_frame = ttk.LabelFrame(main_frame, text="Info")
        info_frame.pack(fill='both', expand=True)
        
        self.info_text = tk.Text(info_frame, height=8, wrap=tk.WORD)
        self.info_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        self.update_info()
    
    def update_range_sensitivity(self, val):
        val = float(val)
        calibration.range_sensitivity = val
        self.range_label.config(text=f"{val:.1f} dB")
    
    def update_motion_sensitivity(self, val):
        val = float(val)
        calibration.motion_sensitivity = val
        self.motion_label.config(text=f"{val:.1f}")
    
    def start_calibration(self):
        calibration.start_calibration()
        self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")
    
    def save_calibration(self):
        calibration.save_calibration()
        messagebox.showinfo("Save", "Calibration saved!")
    
    def load_calibration(self):
        if calibration.load_calibration():
            self.range_var.set(calibration.range_sensitivity)
            self.motion_var.set(calibration.motion_sensitivity)
            self.range_label.config(text=f"{calibration.range_sensitivity:.1f} dB")
            self.motion_label.config(text=f"{calibration.motion_sensitivity:.1f}")
            self.update_info()
            messagebox.showinfo("Load", "Calibration loaded!")
        else:
            messagebox.showerror("Load", "Failed to load!")
    
    def update_status(self, presence, motion, motion_level):
        # Just store the latest values - fast GUI timer will update display
        self.latest_presence = presence
        self.latest_motion = motion
        self.latest_motion_level = motion_level
    
    def fast_update_gui(self):
        """Fast GUI updates independent of plot updates"""
        # Update detection status
        self.presence_var.set("YES" if self.latest_presence else "NO")
        self.motion_var.set("YES" if self.latest_motion else "NO")
        self.motion_level_var.set(f"{self.latest_motion_level:.3f}")
        
        # Update calibration status
        if calibration.is_calibrating:
            self.cal_status.config(text="Ready to calibrate EMPTY ROOM...")
        elif calibration.range_baseline is not None:
            self.cal_status.config(text="✓ Calibrated (Empty Room)")
        else:
            self.cal_status.config(text="Not calibrated")
        
        # Schedule next fast update (much faster than plot updates)
        self.window.after(100, self.fast_update_gui)
    
    def update_info(self):
        info = "Calibration Status:\n\n"
        if calibration.range_baseline is not None:
            info += f"✓ Empty room range baseline: {calibration.range_baseline:.1f} dB\n"
            info += f"✓ Empty room doppler baseline: {calibration.doppler_baseline:.1f} dB\n\n"
            info += "Settings:\n"
            info += f"• Presence sensitivity: {calibration.range_sensitivity:.1f} dB\n"
            info += f"• Motion sensitivity: {calibration.motion_sensitivity:.1f} dB\n\n"
            info += "Detection logic:\n"
            info += "• Presence = range energy ABOVE empty baseline\n"
            info += "• Motion = doppler energy ABOVE empty baseline\n"
            info += "• Range bins 10-50 (avoiding DC)\n"
            info += "• Doppler edges vs center"
        else:
            info += "✗ Not calibrated\n\n"
            info += "Instructions:\n"
            info += "1. LEAVE THE ROOM (empty room)\n"
            info += "2. Click 'Start Calibration'\n"
            info += "3. Come back and adjust sensitivity\n"
            info += "4. Save calibration when satisfied\n\n"
            info += "⚠️ IMPORTANT: Calibrate with EMPTY room!\n"
            info += "Presence/motion detected as increases above empty baseline."
        
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(1.0, info)

# Create calibration control window
def start_gui():
    control = CalibrationControl()
    return control

gui_thread = threading.Thread(target=start_gui, daemon=True)
control_gui = start_gui()  # Create on main thread for macOS compatibility

# Original serial setup (unchanged)
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    exit()

# Original plot setup (unchanged)
rx_frame_channels = [
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32),
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32),
    np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32)
]
ANTENNA_INDEX_TO_DISPLAY = 0

plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(12, 20))

# Plot 1: Raw data
raw_data_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_raw = axs[0].imshow(np.zeros(raw_data_shape), cmap='viridis', aspect='auto',
                        extent=[0, NUM_SAMPLES_PER_CHIRP, 0, NUM_CHIRPS_PER_FRAME])
cb_raw = plt.colorbar(img_raw, ax=axs[0], label='Magnitude')
axs[0].set_xlabel('Sample Index')
axs[0].set_ylabel('Chirp Index')
axs[0].set_title(f'Raw Data (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 2: Range-FFT
range_fft_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_range_fft = axs[1].imshow(np.zeros(range_fft_shape), cmap='viridis', aspect='auto',
                              extent=[-NUM_SAMPLES_PER_CHIRP // 2, NUM_SAMPLES_PER_CHIRP // 2, 0, NUM_CHIRPS_PER_FRAME])
cb_range_fft = plt.colorbar(img_range_fft, ax=axs[1], label='Magnitude (dB)')
axs[1].set_xlabel('Range Bin (Shifted)')
axs[1].set_ylabel('Chirp Index')
axs[1].set_title(f'Range-FFT (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 3: Range-doppler
doppler_fft_shape = (NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)
img_doppler_fft = axs[2].imshow(np.zeros(doppler_fft_shape), cmap='viridis', aspect='auto',
                                extent=[0, NUM_SAMPLES_PER_CHIRP, -NUM_CHIRPS_PER_FRAME // 2, NUM_CHIRPS_PER_FRAME // 2])
cb_doppler_fft = plt.colorbar(img_doppler_fft, ax=axs[2], label='Magnitude (dB)')
axs[2].set_xlabel('Range Bin')
axs[2].set_ylabel('Doppler Bin')
axs[2].set_title(f'Range-Doppler (RX Antenna {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 4: Detection status
axs[3].text(0.1, 0.8, 'Detection Status', fontsize=16, weight='bold')
axs[3].text(0.1, 0.6, 'Use Control Window', fontsize=14)
axs[3].set_xlim(0, 1)
axs[3].set_ylim(0, 1)
axs[3].axis('off')

fig.tight_layout(pad=3.0)

# Original windowing functions (unchanged)
range_window = np.hanning(NUM_SAMPLES_PER_CHIRP)
doppler_window = np.hanning(NUM_CHIRPS_PER_FRAME)

# Original parse function (unchanged)
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
                    continue
    except serial.SerialTimeoutException:
        return None, None, False
    except Exception as e:
        print(f"An error occurred in parse_frame_data_from_serial: {e}")
        if ser and ser.is_open:
            ser.reset_input_buffer()
        return None, None, False

print("Starting radar with simple calibration...")
print("IMPORTANT: Leave room EMPTY, then click 'Start Calibration'")
print("Presence/motion will be detected as increases above empty room baseline")
print("Use the Control Window to calibrate and adjust sensitivity")

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
            print(f"Error: Unexpected frame shape {selected_antenna_frame_data.shape}")
            continue

        # Original processing (unchanged)
        mean_per_chirp = np.mean(selected_antenna_frame_data, axis=1, keepdims=True)
        data_after_chirp_dc_removal = selected_antenna_frame_data - mean_per_chirp

        # Plot 1: Raw Data
        raw_data_to_plot = np.abs(data_after_chirp_dc_removal)
        img_raw.set_data(raw_data_to_plot)
        raw_min = np.percentile(raw_data_to_plot.flatten(), 1)
        raw_max = np.percentile(raw_data_to_plot.flatten(), 99)
        if raw_min >= raw_max: raw_max = raw_min + 1e-3
        img_raw.set_clim(vmin=raw_min, vmax=raw_max)
        axs[0].set_title(f'Raw Data (RX Ant {ANTENNA_INDEX_TO_DISPLAY + 1}, Frame: {parsed_frame_num})')

        # Original signal processing (unchanged)
        windowed_for_range = data_after_chirp_dc_removal * range_window[np.newaxis, :]
        range_fft_output = np.fft.fft(windowed_for_range, axis=1)
        range_fft_shifted_for_plot = np.fft.fftshift(range_fft_output, axes=1)
        range_fft_db = 20 * np.log10(np.abs(range_fft_shifted_for_plot) + 1e-9)
        
        img_range_fft.set_data(range_fft_db)
        rfft_min = np.percentile(range_fft_db.flatten(), 5)
        rfft_max = np.percentile(range_fft_db.flatten(), 99)
        if rfft_min >= rfft_max: rfft_max = rfft_min + 1
        img_range_fft.set_clim(vmin=rfft_min, vmax=rfft_max)
        axs[1].set_title(f'Range-FFT (Frame: {parsed_frame_num})')

        mean_per_range_bin = np.mean(range_fft_output, axis=0, keepdims=True)
        range_fft_dc_removed_for_doppler = range_fft_output - mean_per_range_bin
        windowed_for_doppler = range_fft_dc_removed_for_doppler * doppler_window[:, np.newaxis]
        doppler_fft_output = np.fft.fft(windowed_for_doppler, axis=0)
        doppler_fft_shifted_for_plot = np.fft.fftshift(doppler_fft_output, axes=0)
        spectrogram_db = 20 * np.log10(np.abs(doppler_fft_shifted_for_plot) + 1e-9)

        img_doppler_fft.set_data(spectrogram_db)
        s_min = np.percentile(spectrogram_db.flatten(), 5)
        s_max = np.percentile(spectrogram_db.flatten(), 99)
        if s_min >= s_max: s_max = s_min + 1
        img_doppler_fft.set_clim(vmin=s_min, vmax=s_max)
        axs[2].set_title(f'Range-Doppler (Frame: {parsed_frame_num})')

        # Calibration - immediate with current frame
        if calibration.is_calibrating:
            finished = calibration.calibrate_with_current_frame(range_fft_db, spectrogram_db)
            if finished:
                control_gui.update_info()
        
        # Detection
        presence, presence_score = calibration.detect_presence(range_fft_db)
        motion_level, motion = calibration.detect_motion(spectrogram_db)
        
        # Update GUI (now stores values for fast updates)
        control_gui.update_status(presence, motion, motion_level)
        
        # Update plot 4
        axs[3].clear()
        axs[3].text(0.1, 0.9, 'Simple Calibration Detection', fontsize=16, weight='bold')
        axs[3].text(0.1, 0.7, f"Presence: {'YES' if presence else 'NO'}", fontsize=12,
                   color='green' if presence else 'red')
        axs[3].text(0.1, 0.6, f"Motion: {'YES' if motion else 'NO'}", fontsize=12,
                   color='orange' if motion else 'blue')
        axs[3].text(0.1, 0.5, f"Motion Level: {motion_level:.3f}", fontsize=12)
        
        if presence and motion:
            status = "LIGHTS_ON"
            color = 'green'
        elif presence and not motion:
            status = "KEEP_CURRENT"
            color = 'blue'
        else:
            status = "LIGHTS_OFF"
            color = 'red'
        
        axs[3].text(0.1, 0.4, f"Status: {status}", fontsize=12, weight='bold', color=color)
        
        cal_status = "Calibrating EMPTY ROOM..." if calibration.is_calibrating else "Calibrated (Empty Room)" if calibration.range_baseline is not None else "Not Calibrated"
        axs[3].text(0.1, 0.3, f"Calibration: {cal_status}", fontsize=12)
        
        axs[3].set_xlim(0, 1)
        axs[3].set_ylim(0, 1)
        axs[3].axis('off')

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