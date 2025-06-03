import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import time
import re
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
from scipy.ndimage import label

# --- Radar Configuration (UPDATED from "new" config) ---
NUM_CHIRPS_PER_FRAME = 16
NUM_SAMPLES_PER_CHIRP = 256
NUM_RX_ANTENNAS = 3
EXPECTED_SAMPLES_IN_FLAT_ARRAY = NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS

SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2' # Make sure this is correct for your system
BAUD_RATE = 921900
SERIAL_TIMEOUT = 0.1  # Serial read timeout in seconds

C_LIGHT = 299792458.0
START_FREQUENCY_HZ = 60e9
END_FREQUENCY_HZ = 62e9
CHIRP_REPETITION_TIME_S = 0.0001935

# --- Performance Tuning ---
TARGET_PLOT_FPS = 15  # Target FPS for plot updates
PLOT_UPDATE_INTERVAL_S = 1.0 / TARGET_PLOT_FPS

# --- Helper function for unique prints (to avoid console spam) ---
_printed_warnings = set()
def print_once(message):
    if message not in _printed_warnings:
        print(message)
        _printed_warnings.add(message)

# --- FFT Spectrum function ---
def fft_spectrum_for_range_plot(data_matrix, window_vector):
    if data_matrix.shape[1] == 0 or window_vector.size == 0:
        return np.zeros_like(data_matrix)

    window_vector_2d = window_vector.reshape(1, -1) if window_vector.ndim == 1 else window_vector

    if window_vector_2d.shape[1] != data_matrix.shape[1]:
        print_once(f"Warning: FFT window size {window_vector_2d.shape} mismatch with data {data_matrix.shape}. Using unwindowed.")
        windowed_data = data_matrix
    else:
        windowed_data = data_matrix * window_vector_2d
    
    fft_output_length = data_matrix.shape[1] # N bins (e.g., NUM_SAMPLES_PER_CHIRP)
    fft_processing_length = data_matrix.shape[1] * 2 # 2N point FFT for better resolution
    
    complex_fft_result = np.fft.fft(windowed_data, n=fft_processing_length, axis=1)
    return complex_fft_result[:, :fft_output_length] # Return first N bins (positive frequencies)


class SimpleCalibration:
    def __init__(self):
        self.range_baseline = None
        self.doppler_baseline = None
        self.is_calibrating = False
        self.range_sensitivity = 8.0
        self.motion_sensitivity = 3.0
        self.min_cluster_size = 4
        print("Simple calibration initialized")

    def start_calibration(self):
        self.is_calibrating = True
        print("Ready to calibrate EMPTY ROOM - will use next frame as NO-PRESENCE baseline...")

    def calibrate_with_current_frame(self, range_fft_db_one_sided, range_doppler_db):
        if not self.is_calibrating: return False
        print("Calibrating EMPTY ROOM baseline with current frame...")
        
        roi_end = min(50, range_fft_db_one_sided.shape[1])
        roi_start = min(10, roi_end -1 if roi_end > 0 else 0)
        if roi_start >= roi_end and range_fft_db_one_sided.shape[1] > 0 : 
            roi_start = 0; roi_end = range_fft_db_one_sided.shape[1]

        range_roi = range_fft_db_one_sided[:, roi_start:roi_end]
        self.range_baseline = np.mean(range_roi) if range_roi.size > 0 else -100 
        if range_roi.size == 0: print_once("Warning: Range ROI for calibration was empty.")

        center_doppler = range_doppler_db.shape[0] // 2
        doppler_roi_start = max(0, center_doppler - 5)
        doppler_roi_end = min(range_doppler_db.shape[0], center_doppler + 5)
        doppler_roi = range_doppler_db[doppler_roi_start:doppler_roi_end, :]
        self.doppler_baseline = np.mean(doppler_roi) if doppler_roi.size > 0 else -100
        if doppler_roi.size == 0: print_once("Warning: Doppler ROI for calibration was empty.")

        self.is_calibrating = False
        print(f"EMPTY ROOM baselines: Range={self.range_baseline:.1f} dB, Doppler={self.doppler_baseline:.1f} dB")
        return True

    def detect_presence(self, range_fft_db_one_sided):
        if self.range_baseline is None: return False, 0.0
        
        roi_end = min(50, range_fft_db_one_sided.shape[1])
        roi_start = min(10, roi_end -1 if roi_end > 0 else 0)
        if roi_start >= roi_end and range_fft_db_one_sided.shape[1] > 0:
             roi_start = 0; roi_end = range_fft_db_one_sided.shape[1]
            
        roi = range_fft_db_one_sided[:, roi_start:roi_end]
        if roi.size == 0: return False, 0.0
            
        current_energy = np.mean(roi)
        energy_increase = current_energy - self.range_baseline
        presence_detected = energy_increase > self.range_sensitivity
        return presence_detected, energy_increase

    def detect_motion(self, range_doppler_db):
        if self.doppler_baseline is None: return 0.0, False

        roi_range_end = min(50, range_doppler_db.shape[1])
        roi_range_start = min(10, roi_range_end -1 if roi_range_end > 0 else 0)
        if roi_range_start >= roi_range_end and range_doppler_db.shape[1] > 0 :
            roi_range_start = 0; roi_range_end = range_doppler_db.shape[1]

        roi = range_doppler_db[:, roi_range_start:roi_range_end].copy()
        if roi.size == 0: return 0.0, False

        center_doppler = roi.shape[0] // 2
        static_zone_start = max(0, center_doppler - 2)
        static_zone_end = min(roi.shape[0], center_doppler + 2)
        roi[static_zone_start:static_zone_end, :] = -100 

        energy_threshold = self.doppler_baseline + self.motion_sensitivity
        motion_mask = roi > energy_threshold
        labeled_mask, num_features = label(motion_mask) 

        if num_features == 0: return 0.0, False
        cluster_sizes = np.bincount(labeled_mask.ravel())[1:] 
        motion_level = np.max(cluster_sizes) if cluster_sizes.size > 0 else 0.0
        for size in cluster_sizes:
            if size >= self.min_cluster_size: return motion_level, True 
        return motion_level, False

    def save_calibration(self, filename="simple_radar_calibration.json"):
        data = {'range_baseline': self.range_baseline, 'doppler_baseline': self.doppler_baseline,
                'range_sensitivity': self.range_sensitivity, 'motion_sensitivity': self.motion_sensitivity,
                'min_cluster_size': self.min_cluster_size}
        with open(filename, 'w') as f: json.dump(data, f, indent=2)
        print(f"Calibration saved to {filename}")

    def load_calibration(self, filename="simple_radar_calibration.json"):
        if not os.path.exists(filename): return False
        try:
            with open(filename, 'r') as f: data = json.load(f)
            self.range_baseline = data.get('range_baseline'); self.doppler_baseline = data.get('doppler_baseline')
            self.range_sensitivity = data.get('range_sensitivity', 8.0)
            self.motion_sensitivity = data.get('motion_sensitivity', 3.0)
            self.min_cluster_size = data.get('min_cluster_size', 4)
            print(f"Calibration loaded from {filename}"); return True
        except Exception as e: print(f"Error loading calibration: {e}"); return False

calibration = SimpleCalibration()

class CalibrationControl:
    def __init__(self):
        self.window = tk.Toplevel()
        self.window.title("Calibration Control")
        self.window.geometry("400x550")
        self.window.attributes('-topmost', True)
        self.presence_var = tk.StringVar(value="NO")
        self.motion_var = tk.StringVar(value="NO")
        self.motion_level_var = tk.StringVar(value="0.0")
        self.latest_presence = False; self.latest_motion = False; self.latest_motion_level = 0.0
        self.setup_gui(); self.fast_update_gui()
    def setup_gui(self):
        main_frame=ttk.Frame(self.window);main_frame.pack(fill='both',expand=True,padx=10,pady=10)
        cal_frame=ttk.LabelFrame(main_frame,text="Calibration");cal_frame.pack(fill='x',pady=(0,10))
        btn_frame=ttk.Frame(cal_frame);btn_frame.pack(fill='x',padx=5,pady=5)
        ttk.Button(btn_frame,text="Start Calibration",command=self.start_calibration).pack(side='left',padx=(0,5))
        ttk.Button(btn_frame,text="Save",command=self.save_calibration).pack(side='left',padx=(0,5))
        ttk.Button(btn_frame,text="Load",command=self.load_calibration).pack(side='left')
        self.cal_status=ttk.Label(cal_frame,text="Ready to calibrate");self.cal_status.pack(pady=5)
        stat_frame=ttk.LabelFrame(main_frame,text="Detection Status");stat_frame.pack(fill='x',pady=(0,10))
        ttk.Label(stat_frame,text="Presence:").grid(row=0,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.presence_var,font=('Arial',10,'bold')).grid(row=0,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Motion:").grid(row=1,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_var,font=('Arial',10,'bold')).grid(row=1,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Largest Cluster:").grid(row=2,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_level_var).grid(row=2,column=1,sticky='w',padx=5,pady=2)
        sens_frame=ttk.LabelFrame(main_frame,text="Sensitivity");sens_frame.pack(fill='x',pady=(0,10))
        ttk.Label(sens_frame,text="Presence Sensitivity (dB):").pack(anchor='w',padx=5,pady=(5,0))
        self.range_var=tk.DoubleVar(value=calibration.range_sensitivity)
        ttk.Spinbox(sens_frame,from_=1.0,to=30.0,increment=0.1,textvariable=self.range_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.range_var.trace_add('write',self.update_sensitivities)
        ttk.Label(sens_frame,text="Motion Energy Threshold (dB):").pack(anchor='w',padx=5,pady=(10,0))
        self.motion_sens_var=tk.DoubleVar(value=calibration.motion_sensitivity)
        ttk.Spinbox(sens_frame,from_=-10.0,to=20.0,increment=0.1,textvariable=self.motion_sens_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.motion_sens_var.trace_add('write',self.update_sensitivities)
        ttk.Label(sens_frame,text="Min Cluster Size (pixels):").pack(anchor='w',padx=5,pady=(10,0))
        self.cluster_size_var=tk.IntVar(value=calibration.min_cluster_size)
        ttk.Spinbox(sens_frame,from_=1,to=100,increment=1,textvariable=self.cluster_size_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.cluster_size_var.trace_add('write',self.update_sensitivities)
        info_frame=ttk.LabelFrame(main_frame,text="Info");info_frame.pack(fill='both',expand=True)
        self.info_text=tk.Text(info_frame,height=8,wrap=tk.WORD);self.info_text.pack(fill='both',expand=True,padx=5,pady=5)
        self.update_info()
    def update_sensitivities(self,*args):
        try:
            calibration.range_sensitivity=self.range_var.get()
            calibration.motion_sensitivity=self.motion_sens_var.get()
            calibration.min_cluster_size=self.cluster_size_var.get()
            self.update_info()
        except(tk.TclError,ValueError):pass 
    def start_calibration(self):calibration.start_calibration();self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")
    def save_calibration(self):calibration.save_calibration();messagebox.showinfo("Save","Calibration saved!")
    def load_calibration(self):
        if calibration.load_calibration():
            self.range_var.set(calibration.range_sensitivity);self.motion_sens_var.set(calibration.motion_sensitivity)
            self.cluster_size_var.set(calibration.min_cluster_size);self.update_info();messagebox.showinfo("Load","Calibration loaded!")
        else:messagebox.showerror("Load","Failed to load!")
    def update_status(self,presence,motion,motion_level):self.latest_presence=presence;self.latest_motion=motion;self.latest_motion_level=motion_level
    def fast_update_gui(self): 
        self.presence_var.set("YES" if self.latest_presence else"NO");self.motion_var.set("YES" if self.latest_motion else"NO")
        self.motion_level_var.set(f"{self.latest_motion_level:.0f}")
        if calibration.is_calibrating:self.cal_status.config(text="Ready to calibrate EMPTY ROOM...")
        elif calibration.range_baseline is not None:self.cal_status.config(text="✓ Calibrated (Empty Room)")
        else:self.cal_status.config(text="Not calibrated")
        if self.window.winfo_exists():self.window.after(100,self.fast_update_gui) 
    def update_info(self):
        info="Calibration Status:\n\n"
        if calibration.range_baseline is not None:
            info+=f"✓ Empty room range baseline: {calibration.range_baseline:.1f} dB\n"
            info+=f"✓ Empty room doppler baseline: {calibration.doppler_baseline:.1f} dB\n\n"
            info+="Settings:\n";info+=f"• Presence sensitivity: {calibration.range_sensitivity:.1f} dB\n"
            info+=f"• Motion energy threshold: {calibration.motion_sensitivity:.1f} dB\n"
            info+=f"• Min cluster size: {calibration.min_cluster_size} pixels\n\n"
            info+="Detection logic:\n";info+="• Presence = AVG range energy > baseline\n"
            info+="• Motion = Find clusters of pixels > energy threshold that are > min cluster size"
        else:
            info+="✗ Not calibrated\n\n";info+="Instructions:\n1. LEAVE THE ROOM\n2. Click 'Start Calibration'\n3. Come back and adjust sensitivity\n4. Save calibration"
        self.info_text.delete(1.0,tk.END);self.info_text.insert(1.0,info)

def start_gui():
    control = CalibrationControl()
    return control

control_gui = start_gui()

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
    time.sleep(1) 
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    if control_gui and control_gui.window.winfo_exists():
        control_gui.window.destroy()
    exit()

ANTENNA_INDEX_TO_DISPLAY = 0 

plt.ion() 
fig, axs = plt.subplots(3, 1, figsize=(10, 15)) 

# --- Axes Calculations ---
dist_points_m = np.array([0.0])
velocity_axis_mps = np.array([0.0])
bandwidth_hz = 0.0
center_freq_hz = 0.0

if NUM_SAMPLES_PER_CHIRP > 0:
    bandwidth_hz = END_FREQUENCY_HZ - START_FREQUENCY_HZ
    if bandwidth_hz > 0:
        range_bin_length_calc = C_LIGHT / (4 * bandwidth_hz) 
        dist_points_m = np.arange(NUM_SAMPLES_PER_CHIRP) * range_bin_length_calc
    else:
        print_once("Warning: Bandwidth is zero, range axis in bins.")
        dist_points_m = np.arange(NUM_SAMPLES_PER_CHIRP) 
else:
    dist_points_m = np.arange(1) 

if NUM_CHIRPS_PER_FRAME > 0 and CHIRP_REPETITION_TIME_S > 0:
    center_freq_hz = (START_FREQUENCY_HZ + END_FREQUENCY_HZ) / 2.0
    if center_freq_hz > 0:
        wavelength_m = C_LIGHT / center_freq_hz
        doppler_freqs_shifted = np.fft.fftshift(np.fft.fftfreq(NUM_CHIRPS_PER_FRAME, d=CHIRP_REPETITION_TIME_S))
        velocity_axis_mps = doppler_freqs_shifted * wavelength_m / 2.0 
    else:
        print_once("Warning: Center freq zero, Doppler axis in bins.")
        velocity_axis_mps = np.arange(-NUM_CHIRPS_PER_FRAME//2, NUM_CHIRPS_PER_FRAME//2 + (NUM_CHIRPS_PER_FRAME%2))
else:
    print_once("Warning: Chirp params invalid, Doppler axis in bins.")
    velocity_axis_mps = np.arange(-NUM_CHIRPS_PER_FRAME//2, NUM_CHIRPS_PER_FRAME//2 + (NUM_CHIRPS_PER_FRAME%2))

if len(velocity_axis_mps) == 0 : velocity_axis_mps = np.array([0.0]) 

# --- Plot Initialization ---
# Plot 0: 1D Range Profile
line_range_profile, = axs[0].plot(dist_points_m, np.full_like(dist_points_m, np.nan))
axs[0].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[0].set_ylabel('Magnitude (dB)')
axs[0].set_title(f'1D Range Profile (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')
axs[0].grid(True)
if len(dist_points_m) > 1:
    axs[0].set_xlim(0, dist_points_m[-1]) 
elif len(dist_points_m) == 1:
     axs[0].set_xlim(0, dist_points_m[0] + 0.1) 
else:
    axs[0].set_xlim(0, 1) 

# Plot 1: Range vs. Chirp
img_range_vs_chirp = axs[1].imshow(np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)),
                                 cmap='viridis', aspect='auto', origin='lower',
                                 extent=[dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else 1.0,
                                         0, NUM_CHIRPS_PER_FRAME-1 if NUM_CHIRPS_PER_FRAME > 0 else 0])
cb_range_vs_chirp = plt.colorbar(img_range_vs_chirp, ax=axs[1], label='Magnitude (dB)')
axs[1].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[1].set_ylabel('Chirp Index')
axs[1].set_title(f'Range vs. Chirp (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 2: Range-Doppler
img_range_doppler = axs[2].imshow(np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP)),
                                 cmap='jet', aspect='auto', origin='lower', 
                                 extent=[dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else 1.0,
                                         velocity_axis_mps[0], velocity_axis_mps[-1] if len(velocity_axis_mps)>1 else 1.0])
cb_range_doppler = plt.colorbar(img_range_doppler, ax=axs[2], label='Magnitude (dB)')
axs[2].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[2].set_ylabel('Velocity (m/s)' if center_freq_hz > 0 else 'Doppler Bin')
axs[2].set_title(f'Range-Doppler (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

fig.tight_layout(pad=2.5) 

# --- Windows for FFT ---
range_window = np.hanning(NUM_SAMPLES_PER_CHIRP) if NUM_SAMPLES_PER_CHIRP > 0 else np.array([])
doppler_window = np.hanning(NUM_CHIRPS_PER_FRAME) if NUM_CHIRPS_PER_FRAME > 0 else np.array([])

def parse_frame_data_from_serial():
    try:
        while plt.fignum_exists(fig.number): 
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: 
                return None, None, False 

            match = re.match(r"Frame (\d+): \[(.*)\]", line)
            if match:
                frame_num_str, data_str = match.groups()
                try:
                    flat_data_1d = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                except ValueError:
                    print_once(f"Warning: ValueError parsing data string: {data_str[:100]}...")
                    continue 

                if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    current_rx_frames = [np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32) for _ in range(NUM_RX_ANTENNAS)]
                    idx = 0
                    all_data_parsed = True
                    for c in range(NUM_CHIRPS_PER_FRAME):
                        for s in range(NUM_SAMPLES_PER_CHIRP):
                            for r_ant in range(NUM_RX_ANTENNAS):
                                if idx < len(flat_data_1d):
                                    current_rx_frames[r_ant][c, s] = flat_data_1d[idx]
                                    idx += 1
                                else:
                                    all_data_parsed = False; break
                            if not all_data_parsed: break
                        if not all_data_parsed: break
                    
                    if all_data_parsed:
                        return current_rx_frames[ANTENNA_INDEX_TO_DISPLAY], int(frame_num_str), False
                else:
                    print_once(f"Data length mismatch: Exp {EXPECTED_SAMPLES_IN_FLAT_ARRAY}, Got {len(flat_data_1d)}")
        return None, None, True 
    except Exception as e:
        print(f"Error in parse_frame_data_from_serial: {e}")
        import traceback
        traceback.print_exc()
        return None, None, False 

print("Starting radar with 3 plots (1D Range, Range-Chirp, Range-Doppler)...")
print("Use the Control Window to calibrate and adjust sensitivity")

last_plot_update_time = time.time() 

try:
    while True:
        current_loop_time = time.time()

        if not plt.fignum_exists(fig.number):
            print("Plot window closed. Exiting.")
            break
        
        selected_antenna_frame_data, parsed_frame_num, should_exit = parse_frame_data_from_serial()
        
        if should_exit: 
            break
        if selected_antenna_frame_data is None:
            plt.pause(0.01) 
            continue

        # --- Data Processing ---
        # DC removal per chirp
        # mean_per_chirp = np.mean(selected_antenna_frame_data, axis=1, keepdims=True)
        # data_after_chirp_dc_removal = selected_antenna_frame_data - mean_per_chirp
        data_after_chirp_dc_removal = selected_antenna_frame_data # MODIFIED: Use raw data

        # Range FFT processing
        range_fft_complex_one_sided = fft_spectrum_for_range_plot(data_after_chirp_dc_removal, range_window)
        range_plot_data_for_cal_and_plot = 20 * np.log10(np.abs(range_fft_complex_one_sided) + 1e-9)

        # 1D Range Profile Data
        one_d_profile_abs = np.mean(np.abs(range_fft_complex_one_sided), axis=0)
        one_d_profile_db = 20 * np.log10(one_d_profile_abs + 1e-9)

        # Doppler Processing: DC removal per range bin
        # mean_per_range_bin = np.mean(range_fft_complex_one_sided, axis=0, keepdims=True)
        # range_fft_dc_removed_for_doppler = range_fft_complex_one_sided - mean_per_range_bin
        range_fft_dc_removed_for_doppler = range_fft_complex_one_sided # MODIFIED: Use non-DC-removed data for Doppler

        windowed_for_doppler = range_fft_dc_removed_for_doppler * doppler_window[:, np.newaxis] 
        doppler_fft_output_complex = np.fft.fft(windowed_for_doppler, axis=0) 
        doppler_fft_shifted_for_plot = np.fft.fftshift(doppler_fft_output_complex, axes=0) 
        spectrogram_db = 20 * np.log10(np.abs(doppler_fft_shifted_for_plot) + 1e-9) 
        
        # --- Plotting and GUI Update Section (timed) ---
        if (current_loop_time - last_plot_update_time) >= PLOT_UPDATE_INTERVAL_S:
            last_plot_update_time = current_loop_time

            # Update 1D Range Profile Plot (axs[0])
            line_range_profile.set_ydata(one_d_profile_db)
            if np.any(np.isfinite(one_d_profile_db)):
                 min_1d = np.min(one_d_profile_db[np.isfinite(one_d_profile_db)])
                 max_1d = np.max(one_d_profile_db[np.isfinite(one_d_profile_db)])
                 axs[0].set_ylim(min_1d - 5, max_1d + 10) 
            else:
                axs[0].set_ylim(-80, -20) 
            axs[0].set_title(f'1D Range Profile (Frame: {parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Update Range vs. Chirp Plot (axs[1])
            img_range_vs_chirp.set_data(range_plot_data_for_cal_and_plot)
            if range_plot_data_for_cal_and_plot.size > 0:
                rvc_min, rvc_max = np.percentile(range_plot_data_for_cal_and_plot, [5, 95])
                img_range_vs_chirp.set_clim(vmin=rvc_min, vmax=rvc_max) 
            axs[1].set_title(f'Range vs. Chirp (Frame: {parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Update Range-Doppler Plot (axs[2])
            img_range_doppler.set_data(spectrogram_db)
            if spectrogram_db.size > 0:
                s_min, s_max = np.percentile(spectrogram_db, [15, 98])
                img_range_doppler.set_clim(vmin=s_min, vmax=s_max) 
            axs[2].set_title(f'Range-Doppler (Frame: {parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Calibration Logic and GUI Updates
            if calibration.is_calibrating:
                if calibration.calibrate_with_current_frame(range_plot_data_for_cal_and_plot, spectrogram_db):
                    if control_gui and control_gui.window.winfo_exists(): control_gui.update_info()
            
            presence, presence_score = calibration.detect_presence(range_plot_data_for_cal_and_plot)
            motion_level, motion = calibration.detect_motion(spectrogram_db)
            
            if control_gui and control_gui.window.winfo_exists():
                control_gui.update_status(presence, motion, motion_level)
                if not calibration.is_calibrating and calibration.range_baseline is not None:
                     control_gui.update_info()

            fig.canvas.draw_idle() 
            fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Program stopped (Ctrl+C).")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    plt.ioff() 
    if 'fig' in locals() and hasattr(fig, 'number') and plt.fignum_exists(fig.number):
        print("Close plot window to exit fully.")
        plt.show(block=True) 
    if control_gui and control_gui.window.winfo_exists():
        control_gui.window.destroy()
    print("Program done.")