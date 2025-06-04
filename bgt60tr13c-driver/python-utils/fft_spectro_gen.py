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
NUM_RX_ANTENNAS = 3 # Note: Filtering and new plot will use ANTENNA_INDEX_TO_DISPLAY
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

# --- Filtering Constants ---
FILTERING_FLOOR_LINEAR = 0.0 # Value to set pixels to if they are below threshold
EPSILON_LOG = 1e-9 # For numerical stability in log10

# --- Home Assistant Configuration & Range Thresholds ---
HA_CONFIG_FILE = 'radar_config.json'

# Default range definitions (from old script concept)
DEFAULT_RANGE_DEFINITIONS = {
    "0.0-0.5m": (0.0, 0.5), "0.5-1.0m": (0.5, 1.0), "1.0-2.0m": (1.0, 2.0),
    "2.0-3.0m": (2.0, 3.0), "3.0-4.0m": (3.0, 4.0), "4.0-5.0m": (4.0, 5.0),
    "5.0-6.0m": (5.0, 6.0), "6.0-7.0m": (6.0, 7.0), "7.0-8.0m": (7.0, 8.0),
    "8.0m+": (8.0, float('inf'))
}
# Default linear magnitude thresholds for Range-Doppler (adapt from old script Ant0 concept)
# These are EXAMPLE values, adjust them based on your radar's typical signal strengths.
DEFAULT_RD_THRESHOLDS_LINEAR = {
    "0.0-0.5m": 50.0, "0.5-1.0m": 30.0, "1.0-2.0m": 30.0,
    "2.0-3.0m": 20.0, "3.0-4.0m": 20.0, "4.0-5.0m": 10.0,
    "5.0-6.0m": 10.0, "6.0-7.0m": 5.0, "7.0-8.0m": 5.0,
    "8.0m+": 2.0
}

# Global config dictionary, initialized with comprehensive defaults
ha_config = {
    "home_assistant_enabled": False,
    "home_assistant_url": "http://homeassistant.local:8123",
    "home_assistant_token": "",
    "home_assistant_light_entity_id": "light.your_light",
    "motion_hold_time_s": 10.0,
    "range_definitions": DEFAULT_RANGE_DEFINITIONS.copy(),
    "range_doppler_thresholds_linear": DEFAULT_RD_THRESHOLDS_LINEAR.copy()
}

def load_ha_config():
    global ha_config
    # Start with a fresh copy of defaults for all fields we manage
    default_full_config = {
        "home_assistant_enabled": False,
        "home_assistant_url": "http://homeassistant.local:8123",
        "home_assistant_token": "",
        "home_assistant_light_entity_id": "light.your_light",
        "motion_hold_time_s": 10.0,
        "range_definitions": DEFAULT_RANGE_DEFINITIONS.copy(),
        "range_doppler_thresholds_linear": DEFAULT_RD_THRESHOLDS_LINEAR.copy()
    }

    if os.path.exists(HA_CONFIG_FILE):
        try:
            with open(HA_CONFIG_FILE, 'r') as f:
                loaded_config = json.load(f)
            
            for key, default_value in default_full_config.items():
                if key in loaded_config:
                    loaded_value = loaded_config[key]
                    if isinstance(default_value, dict) and isinstance(loaded_value, dict):
                        # Merge dictionaries: keys in loaded_config take precedence
                        # For range_definitions, ensure all default keys are present
                        if key == "range_definitions":
                            merged_dict = default_value.copy() # Start with defaults
                            merged_dict.update(loaded_value) # Update with loaded
                            default_full_config[key] = merged_dict
                        elif key == "range_doppler_thresholds_linear":
                            merged_dict = default_value.copy()
                            merged_dict.update(loaded_value)
                            # Ensure all keys from current range_definitions are in thresholds
                            # Use the range_definitions that will be active (either default or loaded)
                            active_range_defs = default_full_config["range_definitions"] # or loaded_config.get("range_definitions", DEFAULT_RANGE_DEFINITIONS)
                            for r_key_def in active_range_defs.keys():
                                if r_key_def not in merged_dict: # If a range exists but threshold missing in loaded
                                    merged_dict[r_key_def] = DEFAULT_RD_THRESHOLDS_LINEAR.get(r_key_def, 0.0)
                            default_full_config[key] = merged_dict
                        else: # Generic dictionary merge
                             default_full_config[key].update(loaded_value)
                    elif isinstance(default_value, dict) and not isinstance(loaded_value, dict):
                        print_once(f"Warning: Config type mismatch for '{key}'. Expected dict, got {type(loaded_value)}. Using default for this key.")
                        # Default value for key is already in default_full_config, so do nothing
                    else: # Not a dict or types match, so direct assignment
                        default_full_config[key] = loaded_value
            
            ha_config = default_full_config # Assign the merged config
            print("Home Assistant and Range Threshold config loaded/merged from radar_config.json")

        except Exception as e:
            print(f"Error loading radar_config.json: {e}. Using default HA and Threshold config.")
            ha_config = default_full_config # Fallback to complete defaults
    else:
        print_once(f"Warning: {HA_CONFIG_FILE} not found. Using default config and trying to save it.")
        ha_config = default_full_config # Use complete defaults
        save_ha_config()

def save_ha_config():
    global ha_config
    try:
        # Ensure range_doppler_thresholds_linear aligns with range_definitions before saving
        active_range_defs = ha_config.get("range_definitions", DEFAULT_RANGE_DEFINITIONS)
        current_thresholds = ha_config.get("range_doppler_thresholds_linear", DEFAULT_RD_THRESHOLDS_LINEAR)
        # Prune thresholds for ranges that no longer exist in definitions
        pruned_thresholds = {r_key: current_thresholds.get(r_key, DEFAULT_RD_THRESHOLDS_LINEAR.get(r_key, 0.0)) 
                             for r_key in active_range_defs.keys()}
        ha_config["range_doppler_thresholds_linear"] = pruned_thresholds

        with open(HA_CONFIG_FILE, 'w') as f:
            json.dump(ha_config, f, indent=4)
        print(f"Home Assistant and Range Threshold config saved to {HA_CONFIG_FILE}")
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
    if data_matrix.shape[1] == 0: # No samples
        return np.zeros_like(data_matrix)
    
    # Ensure window_vector is 2D and compatible or handle empty window_vector
    if window_vector.size == 0:
        if data_matrix.shape[1] > 0: # Data exists, apply no window (rectangular)
             window_vector_2d = np.ones((1, data_matrix.shape[1]))
        else: # No data and no window
            return np.zeros_like(data_matrix)
    else:
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

WINDOW_FUNCTIONS = ["Hanning", "Hamming", "Blackman", "Bartlett", "Rectangular"]

def get_window_array(window_name, num_points):
    if num_points <= 0: return np.array([])
    if window_name == "Hanning": return np.hanning(num_points)
    elif window_name == "Hamming": return np.hamming(num_points)
    elif window_name == "Blackman": return np.blackman(num_points)
    elif window_name == "Bartlett": return np.bartlett(num_points)
    elif window_name == "Rectangular": return np.ones(num_points)
    else: print_once(f"Warning: Unknown window name '{window_name}'. Defaulting to Hanning."); return np.hanning(num_points)

class SimpleCalibration:
    def __init__(self):
        self.range_baseline = None
        self.doppler_baseline = None
        self.is_calibrating = False
        self.range_sensitivity = 8.0 # dB
        self.motion_sensitivity = 3.0 # dB
        self.min_cluster_size = 4
        self.baseline_spectrogram_linear = None # For filtered Doppler plot
        print("Simple calibration initialized")

    def start_calibration(self):
        self.is_calibrating = True
        self.baseline_spectrogram_linear = None # Clear old linear baseline on new calibration start
        print("Ready to calibrate EMPTY ROOM - will use next frame as NO-PRESENCE baseline...")

    def calibrate_with_current_frame(self, range_fft_db_one_sided, range_doppler_db_current_frame):
        if not self.is_calibrating: return False
        print("Calibrating EMPTY ROOM baseline with current frame...")
        
        roi_end = min(50, range_fft_db_one_sided.shape[1])
        roi_start = min(10, roi_end -1 if roi_end > 0 else 0)
        if roi_start >= roi_end and range_fft_db_one_sided.shape[1] > 0 : 
            roi_start = 0; roi_end = range_fft_db_one_sided.shape[1]

        range_roi = range_fft_db_one_sided[:, roi_start:roi_end]
        self.range_baseline = np.mean(range_roi) if range_roi.size > 0 else -100 
        if range_roi.size == 0: print_once("Warning: Range ROI for calibration was empty.")

        center_doppler = range_doppler_db_current_frame.shape[0] // 2
        doppler_roi_start = max(0, center_doppler - 5)
        doppler_roi_end = min(range_doppler_db_current_frame.shape[0], center_doppler + 5)
        doppler_roi = range_doppler_db_current_frame[doppler_roi_start:doppler_roi_end, :]
        self.doppler_baseline = np.mean(doppler_roi) if doppler_roi.size > 0 else -100
        if doppler_roi.size == 0: print_once("Warning: Doppler ROI for calibration was empty.")

        # Store baseline for filtered Doppler plot
        self.baseline_spectrogram_linear = 10**(range_doppler_db_current_frame / 20.0)
        print_once("Linear baseline for filtered Doppler plot stored.")

        self.is_calibrating = False
        print(f"EMPTY ROOM baselines: Range={self.range_baseline:.1f} dB, Doppler={self.doppler_baseline:.1f} dB")
        return True

    def detect_presence(self, range_fft_db_one_sided):
        if self.range_baseline is None: return False, 0.0
        roi_end = min(50, range_fft_db_one_sided.shape[1]); roi_start = min(10, roi_end -1 if roi_end > 0 else 0)
        if roi_start >= roi_end and range_fft_db_one_sided.shape[1] > 0: roi_start = 0; roi_end = range_fft_db_one_sided.shape[1]
        roi = range_fft_db_one_sided[:, roi_start:roi_end]
        if roi.size == 0: return False, 0.0
        current_energy = np.mean(roi); energy_increase = current_energy - self.range_baseline
        return energy_increase > self.range_sensitivity, energy_increase

    def detect_motion(self, range_doppler_db):
        if self.doppler_baseline is None: return 0.0, False
        roi_range_end = min(50, range_doppler_db.shape[1]); roi_range_start = min(10, roi_range_end -1 if roi_range_end > 0 else 0)
        if roi_range_start >= roi_range_end and range_doppler_db.shape[1] > 0 : roi_range_start = 0; roi_range_end = range_doppler_db.shape[1]
        roi = range_doppler_db[:, roi_range_start:roi_range_end].copy()
        if roi.size == 0: return 0.0, False
        center_doppler = roi.shape[0] // 2
        static_zone_start = max(0, center_doppler - 2); static_zone_end = min(roi.shape[0], center_doppler + 2)
        roi[static_zone_start:static_zone_end, :] = -100 # Effectively ignore static objects for motion
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
                'min_cluster_size': self.min_cluster_size,
                # Note: baseline_spectrogram_linear is not saved. It's recalibrated live.
                }
        with open(filename, 'w') as f: json.dump(data, f, indent=2)
        print(f"Calibration parameters saved to {filename}")

    def load_calibration(self, filename="simple_radar_calibration.json"):
        if not os.path.exists(filename): return False
        try:
            with open(filename, 'r') as f: data = json.load(f)
            self.range_baseline = data.get('range_baseline'); self.doppler_baseline = data.get('doppler_baseline')
            self.range_sensitivity = data.get('range_sensitivity', 8.0)
            self.motion_sensitivity = data.get('motion_sensitivity', 3.0)
            self.min_cluster_size = data.get('min_cluster_size', 4)
            self.baseline_spectrogram_linear = None # Not loaded, will be set on new calibration.
            print(f"Calibration parameters loaded from {filename}"); return True
        except Exception as e: print(f"Error loading calibration parameters: {e}"); return False

calibration = SimpleCalibration()

class CalibrationControl:
    def __init__(self):
        self.window = tk.Toplevel()
        self.window.title("Calibration & HA Control")
        # Dynamically calculate height based on number of range definitions
        self.window.geometry(f"500x{self.calculate_optimal_height()}") 
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

        # Windowing GUI Vars
        self.range_window_type_var = tk.StringVar(value="Hanning")
        self.doppler_window_type_var = tk.StringVar(value="Hanning")

        # For Range-Doppler Thresholds
        self.rd_threshold_vars = {} # Dict to store tk.DoubleVar for each range threshold

        self.setup_gui()
        self.fast_update_gui()
        self.update_rd_threshold_gui_from_config() # Populate GUI with loaded/default thresholds

    def calculate_optimal_height(self):
        base_height = 380 # Approximate height of existing frames before Info
        num_ranges = len(ha_config.get("range_definitions", DEFAULT_RANGE_DEFINITIONS))
        threshold_section_height_per_item = 25 
        threshold_section_header_height = 20 
        threshold_section_visible_items = 6 # Show this many items before scrolling typically
        threshold_section_display_height = min(num_ranges, threshold_section_visible_items) * threshold_section_height_per_item + threshold_section_header_height
        
        info_frame_height = 150 # Approximate height for info text area
        total_height = base_height + threshold_section_display_height + info_frame_height + 100 # Padding
        return max(850, min(total_height, 1000)) # Min height 850, max 1000

    def setup_gui(self):
        main_frame=ttk.Frame(self.window);main_frame.pack(fill='both',expand=True,padx=10,pady=10)
        
        # Calibration Frame
        cal_frame=ttk.LabelFrame(main_frame,text="Radar Calibration");cal_frame.pack(fill='x',pady=(0,5), ipady=2)
        btn_frame=ttk.Frame(cal_frame);btn_frame.pack(fill='x',padx=5,pady=5)
        ttk.Button(btn_frame,text="Start Calibration",command=self.start_calibration).pack(side='left',padx=(0,5))
        ttk.Button(btn_frame,text="Save Cal Params",command=self.save_calibration).pack(side='left',padx=(0,5)) # Renamed for clarity
        ttk.Button(btn_frame,text="Load Cal Params",command=self.load_calibration).pack(side='left') # Renamed
        self.cal_status=ttk.Label(cal_frame,text="Ready to calibrate");self.cal_status.pack(pady=5)
        
        # Detection Status Frame
        stat_frame=ttk.LabelFrame(main_frame,text="Detection Status");stat_frame.pack(fill='x',pady=(0,5), ipady=2)
        ttk.Label(stat_frame,text="Presence:").grid(row=0,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.presence_var,font=('Arial',10,'bold')).grid(row=0,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Motion:").grid(row=1,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_var,font=('Arial',10,'bold')).grid(row=1,column=1,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,text="Largest Cluster:").grid(row=2,column=0,sticky='w',padx=5,pady=2)
        ttk.Label(stat_frame,textvariable=self.motion_level_var).grid(row=2,column=1,sticky='w',padx=5,pady=2)

        # Sensitivity Frame
        sens_frame=ttk.LabelFrame(main_frame,text="Radar Sensitivity");sens_frame.pack(fill='x',pady=(0,5), ipady=2)
        ttk.Label(sens_frame,text="Presence Sensitivity (dB above baseline):").pack(anchor='w',padx=5,pady=(5,0))
        self.range_var=tk.DoubleVar(value=calibration.range_sensitivity)
        ttk.Spinbox(sens_frame,from_=1.0,to=30.0,increment=0.1,textvariable=self.range_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.range_var.trace_add('write',self.update_sensitivities)
        ttk.Label(sens_frame,text="Motion Energy Threshold (dB above baseline):").pack(anchor='w',padx=5,pady=(5,0)) # Corrected label wording
        self.motion_sens_var=tk.DoubleVar(value=calibration.motion_sensitivity)
        ttk.Spinbox(sens_frame,from_=-10.0,to=20.0,increment=0.1,textvariable=self.motion_sens_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.motion_sens_var.trace_add('write',self.update_sensitivities)
        ttk.Label(sens_frame,text="Min Motion Cluster Size (pixels):").pack(anchor='w',padx=5,pady=(5,0)) # Corrected label wording
        self.cluster_size_var=tk.IntVar(value=calibration.min_cluster_size)
        ttk.Spinbox(sens_frame,from_=1,to=100,increment=1,textvariable=self.cluster_size_var,command=self.update_sensitivities,width=8).pack(anchor='w',padx=5,pady=2)
        self.cluster_size_var.trace_add('write',self.update_sensitivities)

        # NEW: Windowing Control Frame
        win_frame = ttk.LabelFrame(main_frame, text="FFT Windowing Control"); win_frame.pack(fill='x', pady=(0, 5), ipady=2)
        ttk.Label(win_frame, text="Range FFT Window:").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        range_win_combo = ttk.Combobox(win_frame, textvariable=self.range_window_type_var, values=WINDOW_FUNCTIONS, state="readonly", width=15)
        range_win_combo.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        range_win_combo.bind("<<ComboboxSelected>>", self.update_window_settings)

        ttk.Label(win_frame, text="Doppler FFT Window:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        doppler_win_combo = ttk.Combobox(win_frame, textvariable=self.doppler_window_type_var, values=WINDOW_FUNCTIONS, state="readonly", width=15)
        doppler_win_combo.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        doppler_win_combo.bind("<<ComboboxSelected>>", self.update_window_settings)

        # --- Range-Doppler Threshold Control Frame ---
        rd_thresh_outer_frame = ttk.LabelFrame(main_frame, text="Range-Doppler Linear Thresholds (Filtered Plot)")
        rd_thresh_outer_frame.pack(fill='x', pady=(0, 5), ipady=2)
        
        # Create a canvas and a vertical scrollbar for the threshold controls
        # Set a fixed height for the canvas to make scrollbar effective if content overflows
        canvas_height = 150 # Adjust as needed, e.g., 5-6 items
        threshold_canvas = tk.Canvas(rd_thresh_outer_frame, height=canvas_height)
        threshold_scrollbar = ttk.Scrollbar(rd_thresh_outer_frame, orient="vertical", command=threshold_canvas.yview)
        scrollable_threshold_frame = ttk.Frame(threshold_canvas)

        scrollable_threshold_frame.bind("<Configure>", lambda e: threshold_canvas.configure(scrollregion=threshold_canvas.bbox("all")))
        threshold_canvas.create_window((0, 0), window=scrollable_threshold_frame, anchor="nw")
        threshold_canvas.configure(yscrollcommand=threshold_scrollbar.set)

        # Populate threshold controls
        # Use ha_config directly here as it's loaded before GUI setup
        current_range_defs_gui = ha_config.get("range_definitions", DEFAULT_RANGE_DEFINITIONS)
        current_rd_thresholds_gui = ha_config.get("range_doppler_thresholds_linear", DEFAULT_RD_THRESHOLDS_LINEAR)

        for i, range_key in enumerate(current_range_defs_gui.keys()): # Iterate in defined order
            ttk.Label(scrollable_threshold_frame, text=f"{range_key}:", width=12).grid(row=i, column=0, sticky='w', padx=5, pady=1)
            
            var = tk.DoubleVar(value=current_rd_thresholds_gui.get(range_key, 1.0))
            self.rd_threshold_vars[range_key] = var
            # Spinbox for linear magnitudes
            spin = ttk.Spinbox(scrollable_threshold_frame, from_=0.0, to=100000.0, increment=0.1, format="%.1f", textvariable=var, width=10,
                               command=lambda k=range_key, v_ref=var: self.update_rd_threshold_config(k, v_ref.get()))
            spin.grid(row=i, column=1, sticky='w', padx=5, pady=1)
            # Using trace_add for immediate updates as user types or spinbox changes via arrow keys
            var.trace_add('write', lambda name, index, mode, k=range_key, v_ref=var: self.update_rd_threshold_config(k, v_ref.get()))
        
        threshold_canvas.pack(side="left", fill="x", expand=True) # Fill x, not both
        threshold_scrollbar.pack(side="right", fill="y")


        # Home Assistant Frame
        ha_frame = ttk.LabelFrame(main_frame, text="Home Assistant Control")
        ha_frame.pack(fill='x', pady=(0, 5), ipady=2)
        ttk.Checkbutton(ha_frame, text="Enable Home Assistant", variable=self.ha_enabled_var, command=self.update_ha_config_from_gui).grid(row=0, column=0, columnspan=2, sticky='w', padx=5, pady=5)
        ttk.Label(ha_frame, text="HA URL:").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_url_var, width=40).grid(row=1, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="HA Token:").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_token_var, width=40, show="*").grid(row=2, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="Light Entity ID:").grid(row=3, column=0, sticky='w', padx=5, pady=2)
        ttk.Entry(ha_frame, textvariable=self.ha_entity_id_var, width=40).grid(row=3, column=1, sticky='ew', padx=5, pady=2)
        ttk.Label(ha_frame, text="Light Hold Time (s):").grid(row=4, column=0, sticky='w', padx=5, pady=2)
        ttk.Spinbox(ha_frame, from_=0.0, to=300.0, increment=1.0, textvariable=self.light_hold_time_var, command=self.update_ha_config_from_gui, width=8).grid(row=4, column=1, sticky='w', padx=5, pady=2)
        
        ttk.Button(ha_frame, text="Save Full Config", command=self.save_ha_config_action).grid(row=5, column=0, columnspan=2, pady=5) # Renamed
        
        # Info Frame
        info_frame=ttk.LabelFrame(main_frame,text="Info");info_frame.pack(fill='both',expand=True, pady=(5,0))
        self.info_text=tk.Text(info_frame,height=12,wrap=tk.WORD);self.info_text.pack(fill='both',expand=True,padx=5,pady=5) # Adjusted height
        self.update_info()


    def update_rd_threshold_gui_from_config(self):
        # This function is called at init to populate GUI from ha_config
        # and can be called if config is reloaded externally (though current flow doesn't do that post-init)
        current_rd_thresholds = ha_config.get("range_doppler_thresholds_linear", DEFAULT_RD_THRESHOLDS_LINEAR)
        for range_key, var in self.rd_threshold_vars.items():
            var.set(current_rd_thresholds.get(range_key, 0.0)) # Default to 0.0 if key somehow missing

    def update_rd_threshold_config(self, range_key, value_from_gui):
        global ha_config
        try:
            # value_from_gui is already a float due to tk.DoubleVar
            # If it were from an Entry, float(value_str) would be needed.
            value = value_from_gui 
            
            # Ensure the nested dict exists
            if "range_doppler_thresholds_linear" not in ha_config:
                ha_config["range_doppler_thresholds_linear"] = DEFAULT_RD_THRESHOLDS_LINEAR.copy()
            
            if range_key in ha_config["range_doppler_thresholds_linear"]:
                 if ha_config["range_doppler_thresholds_linear"][range_key] != value:
                    ha_config["range_doppler_thresholds_linear"][range_key] = value
                    self.update_info() # Update info panel to reflect the change
            else: # Should not happen if GUI is built from range_definitions
                ha_config["range_doppler_thresholds_linear"][range_key] = value
                self.update_info()


        except ValueError:
            # This might happen if trace_add passes a non-float string, though DoubleVar should prevent it.
            print_once(f"Invalid R-D threshold value for {range_key}: {value_from_gui}")


    def update_ha_config_from_gui(self, *args): # Also called by HA controls
        global ha_config
        ha_config["home_assistant_enabled"] = self.ha_enabled_var.get()
        ha_config["home_assistant_url"] = self.ha_url_var.get()
        ha_config["home_assistant_token"] = self.ha_token_var.get()
        ha_config["home_assistant_light_entity_id"] = self.ha_entity_id_var.get()
        ha_config["motion_hold_time_s"] = self.light_hold_time_var.get()
        # R-D thresholds are updated by their own callbacks `update_rd_threshold_config`
        self.update_info()

    def save_ha_config_action(self):
        # Ensure current GUI values for HA specific parts are in ha_config
        # R-D thresholds are already updated in ha_config by their spinbox/var traces
        self.update_ha_config_from_gui() 
        save_ha_config() # This now saves the entire ha_config, including R-D thresholds
        messagebox.showinfo("Save", "Full configuration (HA & Thresholds) saved!")

    def update_sensitivities(self,*args):
        try:
            calibration.range_sensitivity=self.range_var.get()
            calibration.motion_sensitivity=self.motion_sens_var.get()
            calibration.min_cluster_size=self.cluster_size_var.get()
            self.update_info()
        except(tk.TclError,ValueError):pass # Catch errors if GUI is not ready
    
    def update_window_settings(self, *args):
        # print_once(f"Range window set to: {self.range_window_type_var.get()}")
        # print_once(f"Doppler window set to: {self.doppler_window_type_var.get()}")
        self.update_info()

    def start_calibration(self):
        calibration.start_calibration()
        self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")
        self.update_info()

    def save_calibration(self): # This saves only simple_radar_calibration.json (dB baselines etc)
        calibration.save_calibration()
        messagebox.showinfo("Save", "Radar calibration parameters (dB baselines) saved!")

    def load_calibration(self): # This loads only simple_radar_calibration.json
        if calibration.load_calibration():
            self.range_var.set(calibration.range_sensitivity)
            self.motion_sens_var.set(calibration.motion_sensitivity)
            self.cluster_size_var.set(calibration.min_cluster_size)
            self.update_info()
            messagebox.showinfo("Load","Radar calibration parameters (dB baselines) loaded!")
        else:
            messagebox.showerror("Load","Failed to load radar calibration parameters!")

    def update_status(self,presence,motion,motion_level):
        self.latest_presence=presence
        self.latest_motion=motion
        self.latest_motion_level=motion_level
    
    def fast_update_gui(self): 
        self.presence_var.set("YES" if self.latest_presence else"NO")
        self.motion_var.set("YES" if self.latest_motion else"NO")
        self.motion_level_var.set(f"{self.latest_motion_level:.0f}")
        if calibration.is_calibrating:
            self.cal_status.config(text="Calibrating EMPTY ROOM with next frame...")
        elif calibration.range_baseline is not None: # Indicates dB baselines are set
             cal_text = "✓ Calibrated (dB Baselines"
             if calibration.baseline_spectrogram_linear is not None:
                 cal_text += " & Linear R-D Baseline)"
             else:
                 cal_text += ")"
             self.cal_status.config(text=cal_text)
        else:
            self.cal_status.config(text="Not calibrated")
        
        if self.window.winfo_exists():
            self.window.after(100,self.fast_update_gui) # Schedule next update

    def update_info(self):
        info_parts = []
        info_parts.append("Calibration Status:")
        if calibration.range_baseline is not None:
            info_parts.append(f"  ✓ Empty room range baseline: {calibration.range_baseline:.1f} dB")
            info_parts.append(f"  ✓ Empty room doppler baseline: {calibration.doppler_baseline:.1f} dB")
            info_parts.append(f"  ✓ R-D Linear Baseline (for Filtered Plot): {'Stored' if calibration.baseline_spectrogram_linear is not None else 'Not Stored'}")
        else:
            info_parts.append("  ✗ Not calibrated")
            info_parts.append("\nInstructions:\n1. LEAVE THE ROOM\n2. Click 'Start Calibration'\n3. Adjust settings\n4. Save Config / Cal Params")
        info_parts.append("\nRadar Sensitivity Settings:")
        info_parts.append(f"  • Presence Sensitivity (dB): {calibration.range_sensitivity:.1f}")
        info_parts.append(f"  • Motion Energy Thresh (dB): {calibration.motion_sensitivity:.1f}")
        info_parts.append(f"  • Min Motion Cluster: {calibration.min_cluster_size} pixels")
        
        info_parts.append("\nFFT Windowing:")
        info_parts.append(f"  • Range FFT Window: {self.range_window_type_var.get()}")
        info_parts.append(f"  • Doppler FFT Window: {self.doppler_window_type_var.get()}")

        info_parts.append("\nRange-Doppler Linear Thresholds (Filtered Plot):")
        current_rd_thresholds_info = ha_config.get("range_doppler_thresholds_linear", {})
        current_range_defs_info = ha_config.get("range_definitions", {})
        if not current_range_defs_info: info_parts.append("  (No range definitions loaded)")
        for r_key in current_range_defs_info.keys(): # Iterate in defined order
            info_parts.append(f"  • {r_key}: {current_rd_thresholds_info.get(r_key, 'N/A'):.2f}")
        
        info_parts.append("\nHome Assistant Status:")
        info_parts.append(f"  • Enabled: {'YES' if ha_config.get('home_assistant_enabled') else 'NO'}")
        info_parts.append(f"  • URL: {ha_config.get('home_assistant_url')}")
        info_parts.append(f"  • Token: {'Set' if ha_config.get('home_assistant_token') else 'Not Set'}")
        info_parts.append(f"  • Entity ID: {ha_config.get('home_assistant_light_entity_id')}")
        info_parts.append(f"  • Light Hold Time: {ha_config.get('motion_hold_time_s'):.1f}s")

        self.info_text.delete(1.0,tk.END)
        self.info_text.insert(1.0, "\n".join(info_parts))

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

ANTENNA_INDEX_TO_DISPLAY = 0 # Current script focuses on one antenna for display

plt.ion() 
# Increased to 4 plots, adjusted figsize. tight_layout will adjust spacing.
fig, axs = plt.subplots(4, 1, figsize=(10, 18)) 

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
    dist_points_m = np.arange(1) # Placeholder if NUM_SAMPLES_PER_CHIRP is 0

if NUM_CHIRPS_PER_FRAME > 0 and CHIRP_REPETITION_TIME_S > 0:
    center_freq_hz = (START_FREQUENCY_HZ + END_FREQUENCY_HZ) / 2.0
    if center_freq_hz > 0:
        wavelength_m = C_LIGHT / center_freq_hz
        # Doppler bins from -Fs/2 to Fs/2, where Fs = 1/CHIRP_REPETITION_TIME_S
        # velocity = (doppler_freq * wavelength) / 2
        doppler_freqs_shifted = np.fft.fftshift(np.fft.fftfreq(NUM_CHIRPS_PER_FRAME, d=CHIRP_REPETITION_TIME_S))
        velocity_axis_mps = doppler_freqs_shifted * wavelength_m / 2.0 
    else:
        print_once("Warning: Center freq zero, Doppler axis in bins.")
        velocity_axis_mps = np.arange(-NUM_CHIRPS_PER_FRAME//2, NUM_CHIRPS_PER_FRAME//2 + (NUM_CHIRPS_PER_FRAME%2)) # bins if no freq
else:
    print_once("Warning: Chirp params invalid, Doppler axis in bins.")
    velocity_axis_mps = np.arange(-NUM_CHIRPS_PER_FRAME//2, NUM_CHIRPS_PER_FRAME//2 + (NUM_CHIRPS_PER_FRAME%2)) # bins

if len(velocity_axis_mps) == 0 : velocity_axis_mps = np.array([0.0]) # Ensure not empty
if len(dist_points_m) == 0: dist_points_m = np.array([0.0])


# --- Plot Initialization ---
# Plot 0: 1D Range Profile
line_range_profile, = axs[0].plot(dist_points_m, np.full_like(dist_points_m, np.nan))
axs[0].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[0].set_ylabel('Magnitude (dB)')
axs[0].set_title(f'1D Range Profile (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')
axs[0].grid(True)
axs[0].set_xlim(dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else (dist_points_m[0] + 0.1 if len(dist_points_m)==1 else 1.0) )

# Plot 1: Range vs. Chirp
img_range_vs_chirp = axs[1].imshow(np.zeros((NUM_CHIRPS_PER_FRAME if NUM_CHIRPS_PER_FRAME >0 else 1, 
                                           NUM_SAMPLES_PER_CHIRP if NUM_SAMPLES_PER_CHIRP > 0 else 1)),
                                 cmap='viridis', aspect='auto', origin='lower',
                                 extent=[dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else 1.0,
                                         0, NUM_CHIRPS_PER_FRAME-1 if NUM_CHIRPS_PER_FRAME > 0 else 0])
cb_range_vs_chirp = plt.colorbar(img_range_vs_chirp, ax=axs[1], label='Magnitude (dB)')
axs[1].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[1].set_ylabel('Chirp Index')
axs[1].set_title(f'Range vs. Chirp (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 2: Range-Doppler (Original)
img_range_doppler = axs[2].imshow(np.zeros((NUM_CHIRPS_PER_FRAME if NUM_CHIRPS_PER_FRAME >0 else 1, 
                                          NUM_SAMPLES_PER_CHIRP if NUM_SAMPLES_PER_CHIRP > 0 else 1)),
                                 cmap='jet', aspect='auto', origin='lower', 
                                 extent=[dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else 1.0,
                                         velocity_axis_mps[0], velocity_axis_mps[-1] if len(velocity_axis_mps)>1 else 1.0])
cb_range_doppler = plt.colorbar(img_range_doppler, ax=axs[2], label='Magnitude (dB)')
axs[2].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[2].set_ylabel('Velocity (m/s)' if center_freq_hz > 0 else 'Doppler Bin')
axs[2].set_title(f'Range-Doppler (Original, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

# Plot 3: Filtered Range-Doppler
img_filtered_range_doppler = axs[3].imshow(np.zeros((NUM_CHIRPS_PER_FRAME if NUM_CHIRPS_PER_FRAME >0 else 1, 
                                                   NUM_SAMPLES_PER_CHIRP if NUM_SAMPLES_PER_CHIRP > 0 else 1)),
                                           cmap='jet', aspect='auto', origin='lower',
                                           extent=[dist_points_m[0], dist_points_m[-1] if len(dist_points_m)>1 else 1.0,
                                                   velocity_axis_mps[0], velocity_axis_mps[-1] if len(velocity_axis_mps)>1 else 1.0])
cb_filtered_range_doppler = plt.colorbar(img_filtered_range_doppler, ax=axs[3], label='Magnitude (dB)')
axs[3].set_xlabel(f'Range (m)' if bandwidth_hz > 0 else 'Range Bin')
axs[3].set_ylabel('Velocity (m/s)' if center_freq_hz > 0 else 'Doppler Bin')
axs[3].set_title(f'Filtered Range-Doppler (Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')


fig.tight_layout(pad=2.0, h_pad=2.5) # Adjusted padding

def parse_frame_data_from_serial():
    try:
        while plt.fignum_exists(fig.number): # Keep trying as long as plot is open
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: # Timeout or empty line
                return None, None, False # Not a critical exit, just no data this attempt

            match = re.match(r"Frame (\d+): \[(.*)\]", line)
            if match:
                frame_num_str, data_str = match.groups()
                try:
                    flat_data_1d = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                except ValueError:
                    print_once(f"Warning: ValueError parsing data string: {data_str[:100]}...")
                    continue # Try next line

                if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    # Reshape data for the specified antenna
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
        # If loop exits because plot window closed
        return None, None, True # Should exit main loop
    except Exception as e: # Catch broader exceptions during serial ops
        print(f"Error in parse_frame_data_from_serial: {e}")
        import traceback
        traceback.print_exc()
        return None, None, False # Don't exit on general error, try to recover or timeout

print("Starting radar with 4 plots (1D Range, Range-Chirp, Orig R-D, Filtered R-D)...")
print("Use the Control Window to calibrate, adjust sensitivity/thresholds & HA settings.")

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
            plt.pause(0.01) # Brief pause if no data, to prevent busy-looping
            # Check for light timeout even if no new radar data
            if light_is_on and ha_config.get("home_assistant_enabled"):
                if (current_loop_time - last_presence_time) > ha_config.get("motion_hold_time_s", 10.0):
                    print_once("HA: Light hold time expired. Turning off.")
                    turn_light_off()
                    light_is_on = False
            continue

        # --- Data Processing ---
        data_after_chirp_dc_removal = selected_antenna_frame_data # Using raw data (DC removal disabled in original)

        current_range_window_type = control_gui.range_window_type_var.get() if control_gui else "Hanning"
        current_doppler_window_type = control_gui.doppler_window_type_var.get() if control_gui else "Hanning"

        active_range_window = get_window_array(current_range_window_type, NUM_SAMPLES_PER_CHIRP)
        active_doppler_window = get_window_array(current_doppler_window_type, NUM_CHIRPS_PER_FRAME)

        range_fft_complex_one_sided = fft_spectrum_for_range_plot(data_after_chirp_dc_removal, active_range_window)
        range_plot_data_for_cal_and_plot = 20 * np.log10(np.abs(range_fft_complex_one_sided) + EPSILON_LOG) # For Range vs Chirp plot

        one_d_profile_abs = np.mean(np.abs(range_fft_complex_one_sided), axis=0)
        one_d_profile_db = 20 * np.log10(one_d_profile_abs + EPSILON_LOG)

        # Doppler Processing (original spectrogram)
        range_fft_for_doppler = range_fft_complex_one_sided # Using non-DC-removed data for Doppler (as per original)
        
        if active_doppler_window.size == range_fft_for_doppler.shape[0]: # Ensure shapes are compatible
             windowed_for_doppler = range_fft_for_doppler * active_doppler_window[:, np.newaxis]
        else: 
            if NUM_CHIRPS_PER_FRAME > 0: # Only warn if expecting a window
                 print_once(f"Warning: Doppler window size mismatch or invalid. Using unwindowed for Doppler.")
            windowed_for_doppler = range_fft_for_doppler

        doppler_fft_output_complex = np.fft.fft(windowed_for_doppler, axis=0) 
        doppler_fft_shifted_for_plot = np.fft.fftshift(doppler_fft_output_complex, axes=0) 
        spectrogram_db = 20 * np.log10(np.abs(doppler_fft_shifted_for_plot) + EPSILON_LOG) 
        
        # --- Filtered Range-Doppler Processing ---
        current_spectrogram_linear = 10**(spectrogram_db / 20.0)
        # Initialize with a value that results in a "floor" or "empty" display
        filtered_spectrogram_linear_processed = np.full_like(current_spectrogram_linear, FILTERING_FLOOR_LINEAR)
        data_to_threshold_linear = None

        if calibration.baseline_spectrogram_linear is not None:
            if calibration.baseline_spectrogram_linear.shape == current_spectrogram_linear.shape:
                subtracted_linear = current_spectrogram_linear - calibration.baseline_spectrogram_linear
                subtracted_linear[subtracted_linear < 0] = 0.0 # Floor at zero
                data_to_threshold_linear = subtracted_linear
            else:
                # print_once("Warning: Baseline shape mismatch for filtered plot. Thresholding current data (no subtraction).")
                data_to_threshold_linear = current_spectrogram_linear.copy()
                data_to_threshold_linear[data_to_threshold_linear < 0] = 0.0 
        else: 
            # print_once("No baseline for filtered plot. Thresholding current frame data directly.")
            data_to_threshold_linear = current_spectrogram_linear.copy()

        if data_to_threshold_linear is not None:
            temp_filtered_linear = data_to_threshold_linear.copy()
            current_range_defs_local = ha_config.get("range_definitions", DEFAULT_RANGE_DEFINITIONS)
            current_rd_thresholds_local = ha_config.get("range_doppler_thresholds_linear", DEFAULT_RD_THRESHOLDS_LINEAR)

            if dist_points_m.shape[0] == temp_filtered_linear.shape[1]:
                for j_bin in range(temp_filtered_linear.shape[1]): # Iterate over range bins (columns)
                    dist_m_current_bin = dist_points_m[j_bin]
                    threshold_val_linear = -1.0 # Sentinel

                    found_key = None
                    for r_key_iter in current_range_defs_local.keys():
                        min_r, max_r = current_range_defs_local[r_key_iter]
                        if min_r <= dist_m_current_bin < max_r:
                            found_key = r_key_iter
                            break
                    
                    if found_key:
                        threshold_val_linear = current_rd_thresholds_local.get(found_key, 0.0)
                    # Check "plus" range if no specific range matched (e.g. exactly on boundary or beyond last specific)
                    elif "8.0m+" in current_range_defs_local and dist_m_current_bin >= current_range_defs_local["8.0m+"][0]:
                         threshold_val_linear = current_rd_thresholds_local.get("8.0m+", 0.0)
                    
                    if threshold_val_linear >= 0: # Apply threshold if one was found
                        column_data = temp_filtered_linear[:, j_bin]
                        temp_filtered_linear[:, j_bin] = np.where(column_data >= threshold_val_linear, column_data, FILTERING_FLOOR_LINEAR)
                    else: 
                        temp_filtered_linear[:, j_bin] = FILTERING_FLOOR_LINEAR
                filtered_spectrogram_linear_processed = temp_filtered_linear
            else:
                # print_once("Warning: dist_points_m does not match spectrogram columns. Filtered plot shows floor value.")
                # filtered_spectrogram_linear_processed remains as FILTERING_FLOOR_LINEAR
                pass
        
        filtered_spectrogram_db = 20 * np.log10(filtered_spectrogram_linear_processed + EPSILON_LOG)
        # --- End Filtered R-D Processing ---
        
        if (current_loop_time - last_plot_update_time) >= PLOT_UPDATE_INTERVAL_S:
            last_plot_update_time = current_loop_time

            # Update 1D Range Profile Plot (axs[0])
            line_range_profile.set_ydata(one_d_profile_db)
            if np.any(np.isfinite(one_d_profile_db)):
                 min_1d = np.min(one_d_profile_db[np.isfinite(one_d_profile_db)])
                 max_1d = np.max(one_d_profile_db[np.isfinite(one_d_profile_db)])
                 axs[0].set_ylim(min_1d - 5, max_1d + 10) 
            else:
                axs[0].set_ylim(-80, -20) # Default if all NaN/Inf
            axs[0].set_title(f'1D Range Profile (F:{parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Update Range vs. Chirp Plot (axs[1])
            img_range_vs_chirp.set_data(range_plot_data_for_cal_and_plot)
            if range_plot_data_for_cal_and_plot.size > 0 and np.any(np.isfinite(range_plot_data_for_cal_and_plot)):
                rvc_min, rvc_max = np.percentile(range_plot_data_for_cal_and_plot[np.isfinite(range_plot_data_for_cal_and_plot)], [5, 95])
                img_range_vs_chirp.set_clim(vmin=rvc_min, vmax=max(rvc_max, rvc_min + 1)) # Ensure max > min 
            axs[1].set_title(f'Range vs. Chirp (F:{parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Update Range-Doppler Plot (Original, axs[2])
            img_range_doppler.set_data(spectrogram_db)
            if spectrogram_db.size > 0 and np.any(np.isfinite(spectrogram_db)):
                s_min, s_max = np.percentile(spectrogram_db[np.isfinite(spectrogram_db)], [15, 98])
                img_range_doppler.set_clim(vmin=s_min, vmax=max(s_max, s_min + 1)) 
            axs[2].set_title(f'Range-Doppler (Orig, F:{parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1})')

            # Update Filtered Range-Doppler Plot (axs[3])
            img_filtered_range_doppler.set_data(filtered_spectrogram_db)
            if filtered_spectrogram_db.size > 0 and np.any(np.isfinite(filtered_spectrogram_db)):
                fs_min, fs_max = np.percentile(filtered_spectrogram_db[np.isfinite(filtered_spectrogram_db)], [5, 99])
                img_filtered_range_doppler.set_clim(vmin=fs_min, vmax=max(fs_max, fs_min + 1))
            else: # Default clim if data is all floor or non-finite
                img_filtered_range_doppler.set_clim(vmin=-120, vmax=-60)
            baseline_status_str = "BL ON" if calibration.baseline_spectrogram_linear is not None else "BL OFF"
            axs[3].set_title(f'Filtered R-D (F:{parsed_frame_num}, Ant {ANTENNA_INDEX_TO_DISPLAY + 1}) {baseline_status_str}')


            # Calibration Logic and GUI Updates
            if calibration.is_calibrating:
                if calibration.calibrate_with_current_frame(range_plot_data_for_cal_and_plot, spectrogram_db): # Pass current spectrogram_db
                    if control_gui and control_gui.window.winfo_exists(): control_gui.update_info()
            
            presence, presence_score = calibration.detect_presence(range_plot_data_for_cal_and_plot)
            # Motion detection uses the original (unfiltered by magnitude thresholds) spectrogram for robustness
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
    
    if control_gui and control_gui.window.winfo_exists():
        control_gui.window.destroy()
    print("Program done.")