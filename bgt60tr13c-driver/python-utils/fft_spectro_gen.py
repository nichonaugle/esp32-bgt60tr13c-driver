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
import requests # Added for Home Assistant

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

# --- Home Assistant Configuration ---
HA_CONFIG_FILE = 'radar_config.json'
ha_config = {
    "home_assistant_enabled": False,
    "home_assistant_url": "http://homeassistant.local:8123",
    "home_assistant_token": "",
    "home_assistant_light_entity_id": "light.your_light",
    "motion_hold_time_s": 10.0 # This will be used as light_hold_time_s
}

def load_ha_config():
    global ha_config
    if os.path.exists(HA_CONFIG_FILE):
        try:
            with open(HA_CONFIG_FILE, 'r') as f:
                loaded_config = json.load(f)
                # Merge with defaults to ensure all keys are present
                for key in ha_config:
                    if key in loaded_config:
                        ha_config[key] = loaded_config[key]
                print("Home Assistant config loaded from radar_config.json")
        except Exception as e:
            print(f"Error loading radar_config.json: {e}. Using default HA config.")
    else:
        print_once(f"Warning: {HA_CONFIG_FILE} not found. Using default HA config and trying to save it.")
        save_ha_config() # Save a default if it doesn't exist

def save_ha_config():
    global ha_config
    try:
        with open(HA_CONFIG_FILE, 'w') as f:
            json.dump(ha_config, f, indent=4)
        print(f"Home Assistant config saved to {HA_CONFIG_FILE}")
    except Exception as e:
        print(f"Error saving {HA_CONFIG_FILE}: {e}")

load_ha_config()

# --- Helper function for unique prints (to avoid console spam) ---
_printed_warnings = set()
def print_once(message):
    if message not in _printed_warnings:
        print(message)
        _printed_warnings.add(message)

# --- FFT Spectrum function ---
def fft_spectrum_for_range_plot(data_matrix, window_vector):
    if data_matrix.shape[1] == 0 or window_vector.size == 0:
        # If window vector is empty but data is not, apply no window (rectangular)
        if data_matrix.shape[1] > 0 and window_vector.size == 0 :
             window_vector_2d = np.ones((1,data_matrix.shape[1]))
        else:
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

# NEW: Windowing functions
WINDOW_FUNCTIONS = ["Hanning", "Hamming", "Blackman", "Bartlett", "Rectangular"]

def get_window_array(window_name, num_points):
    if num_points <= 0: return np.array([])
    if window_name == "Hanning":
        return np.hanning(num_points)
    elif window_name == "Hamming":
        return np.hamming(num_points)
    elif window_name == "Blackman":
        return np.blackman(num_points)
    elif window_name == "Bartlett":
        return np.bartlett(num_points)
    elif window_name == "Rectangular":
        return np.ones(num_points)
    else: # Default to Hanning if name is unknown
        print_once(f"Warning: Unknown window name '{window_name}'. Defaulting to Hanning.")
        return np.hanning(num_points)

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
        self.window.title("Calibration & HA Control")
        self.window.geometry("450x850") # MODIFIED: Increased height for windowing controls
        self.window.attributes('-topmost', True)
        self.presence_var = tk.StringVar(value="NO")
        self.motion_var = tk.StringVar(value="NO")
        self.motion_level_var = tk.StringVar(value="0.0")
        self.latest_presence = False; self.latest_motion = False; self.latest_motion_level = 0.0
        
        # HA GUI Vars
        self.ha_enabled_var = tk.BooleanVar(value=ha_config.get("home_assistant_enabled", False))
        self.ha_url_var = tk.StringVar(value=ha_config.get("home_assistant_url", ""))
        self.ha_token_var = tk.StringVar(value=ha_config.get("home_assistant_token", ""))
        self.ha_entity_id_var = tk.StringVar(value=ha_config.get("home_assistant_light_entity_id", ""))
        self.light_hold_time_var = tk.DoubleVar(value=ha_config.get("motion_hold_time_s", 10.0))

        # NEW: Windowing GUI Vars
        self.range_window_type_var = tk.StringVar(value="Hanning")
        self.doppler_window_type_var = tk.StringVar(value="Hanning")


        self.setup_gui(); self.fast_update_gui()

    def setup_gui(self):
        main_frame=ttk.Frame(self.window);main_frame.pack(fill='both',expand=True,padx=10,pady=10)
        
        # Calibration Frame
        cal_frame=ttk.LabelFrame(main_frame,text="Radar Calibration");cal_frame.pack(fill='x',pady=(0,10))
        btn_frame=ttk.Frame(cal_frame);btn_frame.pack(fill='x',padx=5,pady=5)
        ttk.Button(btn_frame,text="Start Calibration",command=self.start_calibration).pack(side='left',padx=(0,5))
        ttk.Button(btn_frame,text="Save Cal",command=self.save_calibration).pack(side='left',padx=(0,5))
        ttk.Button(btn_frame,text="Load Cal",command=self.load_calibration).pack(side='left')
        self.cal_status=ttk.Label(cal_frame,text="Ready to calibrate");self.cal_status.pack(pady=5)
        
        # Detection Status Frame
        stat_frame=ttk.LabelFrame(main_frame,text="Detection Status");stat_frame.pack(fill='x',pady=(0,10))
        ttk.Label(stat_frame,text="Presence:").grid(row=0,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.presence_var,font=('Arial',10,'bold')).grid(row=0,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Motion:").grid(row=1,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_var,font=('Arial',10,'bold')).grid(row=1,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Largest Cluster:").grid(row=2,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_level_var).grid(row=2,column=1,sticky='w',padx=5,pady=2)
        
        # Sensitivity Frame
        sens_frame=ttk.LabelFrame(main_frame,text="Radar Sensitivity");sens_frame.pack(fill='x',pady=(0,10))
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

        # NEW: Windowing Control Frame
        win_frame = ttk.LabelFrame(main_frame, text="FFT Windowing Control")
        win_frame.pack(fill='x', pady=(0, 10))
        ttk.Label(win_frame, text="Range FFT Window:").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        range_win_combo = ttk.Combobox(win_frame, textvariable=self.range_window_type_var, values=WINDOW_FUNCTIONS, state="readonly", width=15)
        range_win_combo.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        range_win_combo.bind("<<ComboboxSelected>>", self.update_window_settings)

        ttk.Label(win_frame, text="Doppler FFT Window:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        doppler_win_combo = ttk.Combobox(win_frame, textvariable=self.doppler_window_type_var, values=WINDOW_FUNCTIONS, state="readonly", width=15)
        doppler_win_combo.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        doppler_win_combo.bind("<<ComboboxSelected>>", self.update_window_settings)


        # Home Assistant Frame
        ha_frame = ttk.LabelFrame(main_frame, text="Home Assistant Control")
        ha_frame.pack(fill='x', pady=(0, 10))
        ttk.Checkbutton(ha_frame, text="Enable Home Assistant", variable=self.ha_enabled_var, command=self.update_ha_config_from_gui).grid(row=0, column=0, columnspan=2, sticky='w', padx=5, pady=5)
        ttk.Label(ha_frame, text="HA URL:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_url_var, width=40).grid(row=1, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="HA Token:").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_token_var, width=40, show="*").grid(row=2, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="Light Entity ID:").grid(row=3, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_entity_id_var, width=40).grid(row=3, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="Light Hold Time (s):").grid(row=4, column=0, sticky='w', padx=5, pady=2)
        ttk.Spinbox(ha_frame, from_=0.0, to=300.0, increment=1.0, textvariable=self.light_hold_time_var, command=self.update_ha_config_from_gui, width=8).grid(row=4, column=1, sticky='w', padx=5, pady=2)
        
        ttk.Button(ha_frame, text="Save HA Config", command=self.save_ha_config_action).grid(row=5, column=0, columnspan=2, pady=5)
        
        # Info Frame
        info_frame=ttk.LabelFrame(main_frame,text="Info");info_frame.pack(fill='both',expand=True)
        self.info_text=tk.Text(info_frame,height=10,wrap=tk.WORD);self.info_text.pack(fill='both',expand=True,padx=5,pady=5) # MODIFIED: Increased height
        self.update_info()

    def update_ha_config_from_gui(self, *args):
        global ha_config
        ha_config["home_assistant_enabled"] = self.ha_enabled_var.get()
        ha_config["home_assistant_url"] = self.ha_url_var.get()
        ha_config["home_assistant_token"] = self.ha_token_var.get()
        ha_config["home_assistant_light_entity_id"] = self.ha_entity_id_var.get()
        ha_config["motion_hold_time_s"] = self.light_hold_time_var.get() # GUI uses light_hold_time_var
        self.update_info()

    def save_ha_config_action(self):
        self.update_ha_config_from_gui() # Ensure current GUI values are in ha_config
        save_ha_config()
        messagebox.showinfo("Save", "Home Assistant configuration saved!")

    def update_sensitivities(self,*args):
        try:
            calibration.range_sensitivity=self.range_var.get()
            calibration.motion_sensitivity=self.motion_sens_var.get()
            calibration.min_cluster_size=self.cluster_size_var.get()
            self.update_info()
        except(tk.TclError,ValueError):pass
    
    # NEW: Callback for window selection change
    def update_window_settings(self, *args):
        print_once(f"Range window set to: {self.range_window_type_var.get()}")
        print_once(f"Doppler window set to: {self.doppler_window_type_var.get()}")
        self.update_info() # Update info panel to reflect changes

    def start_calibration(self):calibration.start_calibration();self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")
    def save_calibration(self):calibration.save_calibration();messagebox.showinfo("Save","Radar calibration saved!")
    def load_calibration(self):
        if calibration.load_calibration():
            self.range_var.set(calibration.range_sensitivity);self.motion_sens_var.set(calibration.motion_sensitivity)
            self.cluster_size_var.set(calibration.min_cluster_size);self.update_info();messagebox.showinfo("Load","Radar calibration loaded!")
        else:messagebox.showerror("Load","Failed to load radar calibration!")

    def update_status(self,presence,motion,motion_level):self.latest_presence=presence;self.latest_motion=motion;self.latest_motion_level=motion_level
    
    def fast_update_gui(self): 
        self.presence_var.set("YES" if self.latest_presence else"NO");self.motion_var.set("YES" if self.latest_motion else"NO")
        self.motion_level_var.set(f"{self.latest_motion_level:.0f}")
        if calibration.is_calibrating:self.cal_status.config(text="Ready to calibrate EMPTY ROOM...")
        elif calibration.range_baseline is not None:self.cal_status.config(text="✓ Calibrated (Empty Room)")
        else:self.cal_status.config(text="Not calibrated")
        if self.window.winfo_exists():self.window.after(100,self.fast_update_gui) 

    def update_info(self):
        info="Calibration Status:\n"
        if calibration.range_baseline is not None:
            info+=f"✓ Empty room range baseline: {calibration.range_baseline:.1f} dB\n"
            info+=f"✓ Empty room doppler baseline: {calibration.doppler_baseline:.1f} dB\n\n"
            info+="Radar Settings:\n";info+=f"• Presence sensitivity: {calibration.range_sensitivity:.1f} dB\n"
            info+=f"• Motion energy threshold: {calibration.motion_sensitivity:.1f} dB\n"
            info+=f"• Min cluster size: {calibration.min_cluster_size} pixels\n\n"
        else:
            info+="✗ Not calibrated\n\n";info+="Instructions:\n1. LEAVE THE ROOM\n2. Click 'Start Calibration'\n3. Come back and adjust sensitivity\n4. Save calibration\n\n"
        
        # NEW: Display selected window types in info
        info += "FFT Windowing:\n"
        info += f"• Range FFT Window: {self.range_window_type_var.get()}\n"
        info += f"• Doppler FFT Window: {self.doppler_window_type_var.get()}\n\n"

        info += "Home Assistant Status:\n"
        info += f"• Enabled: {'YES' if ha_config.get('home_assistant_enabled') else 'NO'}\n"
        info += f"• URL: {ha_config.get('home_assistant_url')}\n"
        info += f"• Token: {'Set' if ha_config.get('home_assistant_token') else 'Not Set'}\n"
        info += f"• Entity ID: {ha_config.get('home_assistant_light_entity_id')}\n"
        info += f"• Light Hold Time: {ha_config.get('motion_hold_time_s'):.1f}s\n"

        self.info_text.delete(1.0,tk.END);self.info_text.insert(1.0,info)

def start_gui():
    control = CalibrationControl()
    return control

# --- Home Assistant Functions ---
def call_home_assistant_service(service, entity_id):
    if not ha_config.get("home_assistant_enabled"):
        return
    
    url = f"{ha_config.get('home_assistant_url', '').rstrip('/')}/api/services/light/{service}"
    headers = {
        "Authorization": f"Bearer {ha_config.get('home_assistant_token', '')}",
        "content-type": "application/json",
    }
    data = {"entity_id": entity_id}
    try:
        response = requests.post(url, headers=headers, json=data, timeout=5)
        response.raise_for_status() # Raise an exception for HTTP errors
        print_once(f"HA: Called {service} for {entity_id}. Status: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print_once(f"HA Error: Could not call {service} for {entity_id}: {e}")

def turn_light_on():
    call_home_assistant_service("turn_on", ha_config.get("home_assistant_light_entity_id"))

def turn_light_off():
    call_home_assistant_service("turn_off", ha_config.get("home_assistant_light_entity_id"))


control_gui = start_gui() # Initialize GUI first so it can load HA config

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

# MODIFIED: These global initializations are now just fallbacks or could be removed
# as the loop will use dynamically generated windows from GUI settings.
# range_window = np.hanning(NUM_SAMPLES_PER_CHIRP) if NUM_SAMPLES_PER_CHIRP > 0 else np.array([])
# doppler_window = np.hanning(NUM_CHIRPS_PER_FRAME) if NUM_CHIRPS_PER_FRAME > 0 else np.array([])

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
print("Use the Control Window to calibrate and adjust sensitivity / Home Assistant settings.")

last_plot_update_time = time.time() 
# --- HA Light Control State ---
light_is_on = False
last_presence_time = 0.0

try:
    while True:
        current_loop_time = time.time()

        if not plt.fignum_exists(fig.number) or (control_gui and not control_gui.window.winfo_exists()):
            print("Plot or Control window closed. Exiting.")
            break
        
        selected_antenna_frame_data, parsed_frame_num, should_exit = parse_frame_data_from_serial()
        
        if should_exit: 
            break
        if selected_antenna_frame_data is None:
            plt.pause(0.01) 
            # Check for light timeout even if no new radar data
            if light_is_on and ha_config.get("home_assistant_enabled"):
                if (current_loop_time - last_presence_time) > ha_config.get("motion_hold_time_s", 10.0):
                    print_once("HA: Light hold time expired. Turning off.")
                    turn_light_off()
                    light_is_on = False
            continue

        # --- Data Processing ---
        # DC removal per chirp (DISABLED)
        # mean_per_chirp = np.mean(selected_antenna_frame_data, axis=1, keepdims=True)
        # data_after_chirp_dc_removal = selected_antenna_frame_data - mean_per_chirp
        data_after_chirp_dc_removal = selected_antenna_frame_data # Using raw data

        # Get current window selections from GUI
        current_range_window_type = control_gui.range_window_type_var.get() if control_gui else "Hanning"
        current_doppler_window_type = control_gui.doppler_window_type_var.get() if control_gui else "Hanning"

        # Generate window arrays based on selection
        active_range_window = get_window_array(current_range_window_type, NUM_SAMPLES_PER_CHIRP)
        active_doppler_window = get_window_array(current_doppler_window_type, NUM_CHIRPS_PER_FRAME)


        # Range FFT processing
        range_fft_complex_one_sided = fft_spectrum_for_range_plot(data_after_chirp_dc_removal, active_range_window)
        range_plot_data_for_cal_and_plot = 20 * np.log10(np.abs(range_fft_complex_one_sided) + 1e-9)

        # 1D Range Profile Data
        one_d_profile_abs = np.mean(np.abs(range_fft_complex_one_sided), axis=0)
        one_d_profile_db = 20 * np.log10(one_d_profile_abs + 1e-9)

        # Doppler Processing: DC removal per range bin (DISABLED)
        # mean_per_range_bin = np.mean(range_fft_complex_one_sided, axis=0, keepdims=True)
        # range_fft_dc_removed_for_doppler = range_fft_complex_one_sided - mean_per_range_bin
        range_fft_dc_removed_for_doppler = range_fft_complex_one_sided # Using non-DC-removed data for Doppler
        
        # MODIFIED: Use active_doppler_window
        if active_doppler_window.size == range_fft_dc_removed_for_doppler.shape[0]: # Ensure shapes are compatible
             windowed_for_doppler = range_fft_dc_removed_for_doppler * active_doppler_window[:, np.newaxis]
        else: # Fallback or error handling if shapes mismatch (e.g. NUM_CHIRPS_PER_FRAME is 0)
            print_once(f"Warning: Doppler window size mismatch. Using unwindowed for Doppler.")
            windowed_for_doppler = range_fft_dc_removed_for_doppler

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
                s_min, s_max = np.percentile(spectrogram_db, [15, 98]) # Adjusted percentiles for potentially better contrast
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


            # --- Home Assistant Light Control ---
            if ha_config.get("home_assistant_enabled"):
                if presence: # Using presence for light trigger as per original logic
                    if not light_is_on:
                        print_once("HA: Presence detected. Turning light on.")
                        turn_light_on()
                        light_is_on = True
                    last_presence_time = current_loop_time
                elif light_is_on:
                    if (current_loop_time - last_presence_time) > ha_config.get("motion_hold_time_s", 10.0):
                        print_once("HA: Light hold time expired. Turning off.")
                        turn_light_off()
                        light_is_on = False
            elif light_is_on: # If HA was disabled while light was on
                print_once("HA: Disabled, turning off light if it was on.")
                turn_light_off() # Attempt to turn off if it was on
                light_is_on = False


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
        plt.show(block=True) # Keep plot open until manually closed
    
    # Ensure GUI is closed if it exists
    if control_gui and control_gui.window.winfo_exists():
        control_gui.window.destroy()
    print("Program done.")