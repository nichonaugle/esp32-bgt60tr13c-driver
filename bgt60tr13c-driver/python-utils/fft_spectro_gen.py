import matplotlib
matplotlib.use('TkAgg') # Ensure this is suitable for your environment
import numpy as np
from scipy.signal import windows
from scipy import constants
import matplotlib.pyplot as plt
import time
import serial
import re
import traceback
import tkinter as tk
from tkinter import ttk, messagebox
import json
import requests
from scipy.ndimage import label

# Radar Configuration
NUM_CHIRPS_PER_FRAME = 16
NUM_SAMPLES_PER_CHIRP = 256
NUM_RX_ANTENNAS = 1
START_FREQUENCY_HZ = 60000000000
END_FREQUENCY_HZ = 62000000000
SAMPLE_RATE_HZ = 2000000
CHIRP_REPETITION_TIME_S = 0.0001935

# Hardcoded Settings
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'
BAUD_RATE = 921900
PROCESSING_FRAMERATE_HZ = 10

EXPECTED_SAMPLES_IN_FLAT_ARRAY = NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS
BINS_TO_SKIP_FOR_PEAK_DETECTION = 5
BINS_TO_IGNORE_FOR_PLOT_YLIM = 5
CALIBRATION_KEY = 'c'
FILTERING_FLOOR_1D_PROFILE_LINEAR = 0.0
FILTERING_FLOOR_2D_MAG_LINEAR = 1e-9
CONFIG_FILE_PATH = "radar_config.json"

class AppState:
    def __init__(self):
        self.calibration_requested = False
        self.filtering_enabled = False
        self.baseline_1d_profiles = [None] * NUM_RX_ANTENNAS
        self.baseline_2d_fft_abs = [None] * NUM_RX_ANTENNAS
        self.baseline_rd_maps_abs = [None] * NUM_RX_ANTENNAS
        self.gui_is_active = False
        self.gui_root = None
        self.auto_calibration_enabled = False
        self.auto_calibration_interval_s = 8.0 # Default, will be overridden by radar_config.json if present
        self.last_calibration_time = 0.0

        # Updated range definitions
        self.range_definitions = {
            "0.0-0.5m": (0.0, 0.5),
            "0.5-1.0m": (0.5, 1.0),
            "1.0-2.0m": (1.0, 2.0),
            "2.0-3.0m": (2.0, 3.0),
            "3.0-4.0m": (3.0, 4.0),
            "4.0-5.0m": (4.0, 5.0),
            "5.0-6.0m": (5.0, 6.0),
            "6.0-7.0m": (6.0, 7.0),
            "7.0-8.0m": (7.0, 8.0),
            "8.0m+": (8.0, float('inf'))
        }

        # <<< ADDED: Color mapping for ranges >>>
        self.range_to_color_rgb = {
            "0.0-0.5m": [255, 0, 0],    # Red
            "0.5-1.0m": [255, 165, 0],  # Orange
            "1.0-2.0m": [255, 255, 0],  # Yellow
            "2.0-3.0m": [0, 255, 0],    # Green
            "3.0-4.0m": [0, 0, 255],    # Blue
            "4.0-5.0m": [75, 0, 130],   # Indigo (Darker Purple)
            "5.0-6.0m": [238, 130, 238],# Violet (Lighter Purple)
            "6.0-7.0m": [0, 255, 255],  # Cyan
            "7.0-8.0m": [255, 0, 255],  # Magenta
            "8.0m+":    [255, 255, 255] # White
        }


        # Default thresholds for 1D profile (per antenna) - updated for new ranges
        default_thresholds_1d_single_antenna = {
            "0.0-0.5m": 0.07, "0.5-1.0m": 0.05, "1.0-2.0m": 0.05,
            "2.0-3.0m": 0.03, "3.0-4.0m": 0.03, "4.0-5.0m": 0.02,
            "5.0-6.0m": 0.02, "6.0-7.0m": 0.015, "7.0-8.0m": 0.015,
            "8.0m+": 0.01
        }
        # Default thresholds for 2D map (per antenna) - updated for new ranges
        # Values for antenna 0
        default_thresholds_2d_antenna0 = {
            "0.0-0.5m": 9200.0, "0.5-1.0m": 3800.0, "1.0-2.0m": 3800.0,
            "2.0-3.0m": 3200.0, "3.0-4.0m": 3200.0, "4.0-5.0m": 2100.0,
            "5.0-6.0m": 2100.0, "6.0-7.0m": 1600.0, "7.0-8.0m": 1600.0,
            "8.0m+": 900.0
        }
        # Values for antenna 1 and 2 (assuming they are similar to each other from original config)
        default_thresholds_2d_antenna1_2 = {
            "0.0-0.5m": 0.007, "0.5-1.0m": 0.005, "1.0-2.0m": 0.005,
            "2.0-3.0m": 0.003, "3.0-4.0m": 0.003, "4.0-5.0m": 0.002,
            "5.0-6.0m": 0.002, "6.0-7.0m": 0.0015, "7.0-8.0m": 0.0015,
            "8.0m+": 0.001
        }
        self.per_antenna_range_thresholds_1d = [default_thresholds_1d_single_antenna.copy() for _ in range(NUM_RX_ANTENNAS)]
        self.per_antenna_range_thresholds_2d = [default_thresholds_2d_antenna0.copy()] + \
                                               [default_thresholds_2d_antenna1_2.copy() for _ in range(NUM_RX_ANTENNAS - 1)]
        if NUM_RX_ANTENNAS == 0 : # Handle case with 0 antennas if it were possible
             self.per_antenna_range_thresholds_2d = []


        # <<< MODIFIED Motion Detection Settings - Now Per Range >>>
        # Default motion pixel magnitude thresholds - updated for new ranges
        default_motion_pixel_mag_thresh = {
            "0.0-0.5m": 0.01, "0.5-1.0m": 0.01, "1.0-2.0m": 0.01,
            "2.0-3.0m": 0.01, "3.0-4.0m": 0.01, "4.0-5.0m": 0.001,
            "5.0-6.0m": 0.001, "6.0-7.0m": 0.0011,"7.0-8.0m": 0.01,
            "8.0m+": 0.01
        }
        # Default motion pixel count thresholds - updated for new ranges
        default_motion_pixel_count_thresh = {
            "0.0-0.5m": 5, "0.5-1.0m": 5, "1.0-2.0m": 3,
            "2.0-3.0m": 3, "3.0-4.0m": 2, "4.0-5.0m": 1,
            "5.0-6.0m": 1, "6.0-7.0m": 1, "7.0-8.0m": 10,
            "8.0m+": 10
        }
        # Default motion consecutive frames thresholds - updated for new ranges
        default_motion_consecutive_frames_thresh = {
            "0.0-0.5m": 3, "0.5-1.0m": 3, "1.0-2.0m": 3,
            "2.0-3.0m": 2, "3.0-4.0m": 2, "4.0-5.0m": 2,
            "5.0-6.0m": 1, "6.0-7.0m": 2, "7.0-8.0m": 2,
            "8.0m+": 2
        }

        self.motion_pixel_magnitude_thresholds = default_motion_pixel_mag_thresh.copy()
        self.motion_pixel_count_thresholds = default_motion_pixel_count_thresh.copy()
        self.motion_consecutive_frames_thresholds = default_motion_consecutive_frames_thresh.copy()
        
        # Overall motion status and per-range tracking
        self.motion_detected_status = False # Overall status
        self.motion_hold_time_s = 8.0 # Overall hold time for global status, default from radar_config.json
        self.motion_last_overall_detected_time = 0.0


        self.motion_detected_status_by_range = {k: False for k in self.range_definitions}
        self.motion_last_detected_time_by_range = {k: 0.0 for k in self.range_definitions}
        self.motion_consecutive_frames_current_count_by_range = {k: 0 for k in self.range_definitions}
        self.motion_current_gap_count_by_range = {k: 0 for k in self.range_definitions}
        self.motion_allowed_gaps = 5 # Global, default from radar_config.json

        # Home Assistant Settings
        self.home_assistant_enabled = True # Default from radar_config.json
        self.home_assistant_url = "http://homeassistant.local:8123" # Default from radar_config.json
        self.home_assistant_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiIxMTM1MjEzZjkxYzI0MzI2Yjg4YTBmYTQ3NGJkYTdjYyIsImlhdCI6MTc0ODc0Njg4MCwiZXhwIjoyMDY0MTA2ODgwfQ.dO68DBHdzv29IMAKemkJeo0T4F67IaH9oA1tm7MF92I" # Default from radar_config.json
        self.home_assistant_light_entity_id = "light.smart_multicolor_bulb" # Default from radar_config.json
        self.light_is_on = False
        self.current_light_color = None # <<< ADDED: Track current light color

        # Sustained Presence Magnification Settings
        self.sustained_presence_enabled = True # Default from radar_config.json
        self.sustained_min_hot_pixels_for_group = 2 # Default from radar_config.json
        self.sustained_pixel_proximity_threshold = 7 # Default from radar_config.json
        self.sustained_duration_threshold_s = 10.0 # Default from radar_config.json
        self.sustained_magnification_factor = 3.0 # Default from radar_config.json
        self.sustained_max_gap_frames = 6 # Default from radar_config.json
        self.sustained_active_groups = {}
        self.next_sustained_group_id = 0
        
        # Penalize New Pixel Clusters Settings
        self.penalize_new_clusters_enabled = True # Default from radar_config.json
        self.penalize_frames_duration = 1 # Default from radar_config.json
        self.penalize_reduction_factor = 0.15 # Default from radar_config.json
        self.new_cluster_penalty_tracker = {}
        self.next_penalty_group_id = 0

        # GUI Variable Storage
        self.gui_auto_cal_enabled_var = None
        self.gui_auto_cal_interval_var = None
        self.gui_motion_vars_by_range = {k: {} for k in self.range_definitions} # <<< MODIFIED for new ranges
        self.gui_motion_generic_vars = {} # For global settings like hold time, allowed gaps
        self.gui_ha_vars = {}
        self.gui_1d_threshold_vars = [{} for _ in range(NUM_RX_ANTENNAS)]
        self.gui_2d_threshold_vars = [{} for _ in range(NUM_RX_ANTENNAS)]
        self.gui_sustained_vars = {}
        self.gui_penalize_vars = {}

        print(f"Press '{CALIBRATION_KEY}' in the plot window to calibrate baseline noise.")
        print("A separate window for filtering, motion, and HA controls will also appear.")

def save_configuration(app_state: AppState, filepath: str):
    config_data = {
        "auto_calibration_enabled": app_state.auto_calibration_enabled,
        "auto_calibration_interval_s": app_state.auto_calibration_interval_s,
        "motion_pixel_magnitude_thresholds": app_state.motion_pixel_magnitude_thresholds,
        "motion_pixel_count_thresholds": app_state.motion_pixel_count_thresholds,
        "motion_consecutive_frames_thresholds": app_state.motion_consecutive_frames_thresholds,
        "motion_hold_time_s": app_state.motion_hold_time_s,
        "motion_allowed_gaps": app_state.motion_allowed_gaps,
        "per_antenna_range_thresholds_1d": app_state.per_antenna_range_thresholds_1d,
        "per_antenna_range_thresholds_2d": app_state.per_antenna_range_thresholds_2d,
        "home_assistant_enabled": app_state.home_assistant_enabled,
        "home_assistant_url": app_state.home_assistant_url,
        "home_assistant_token": app_state.home_assistant_token,
        "home_assistant_light_entity_id": app_state.home_assistant_light_entity_id,
        "sustained_presence_enabled": app_state.sustained_presence_enabled,
        "sustained_min_hot_pixels_for_group": app_state.sustained_min_hot_pixels_for_group,
        "sustained_pixel_proximity_threshold": app_state.sustained_pixel_proximity_threshold,
        "sustained_duration_threshold_s": app_state.sustained_duration_threshold_s,
        "sustained_magnification_factor": app_state.sustained_magnification_factor,
        "sustained_max_gap_frames": app_state.sustained_max_gap_frames,
        "penalize_new_clusters_enabled": app_state.penalize_new_clusters_enabled,
        "penalize_frames_duration": app_state.penalize_frames_duration,
        "penalize_reduction_factor": app_state.penalize_reduction_factor,
    }
    try:
        with open(filepath, 'w') as f:
            json.dump(config_data, f, indent=4)
        print(f"Configuration saved to {filepath}")
        if app_state.gui_root and app_state.gui_is_active:
             messagebox.showinfo("Configuration", f"Configuration saved to {filepath}", parent=app_state.gui_root)
    except Exception as e:
        print(f"Error saving configuration: {e}")
        if app_state.gui_root and app_state.gui_is_active:
            messagebox.showerror("Configuration Error", f"Error saving configuration: {e}", parent=app_state.gui_root)

def load_configuration(app_state: AppState, filepath: str):
    try:
        with open(filepath, 'r') as f:
            config_data = json.load(f)

        app_state.auto_calibration_enabled = config_data.get("auto_calibration_enabled", app_state.auto_calibration_enabled)
        app_state.auto_calibration_interval_s = config_data.get("auto_calibration_interval_s", app_state.auto_calibration_interval_s)

        config_motion_mag_thresholds = config_data.get("motion_pixel_magnitude_thresholds", {})
        for r_key_new in app_state.range_definitions:
            app_state.motion_pixel_magnitude_thresholds[r_key_new] = config_motion_mag_thresholds.get(r_key_new, app_state.motion_pixel_magnitude_thresholds[r_key_new])

        config_motion_count_thresholds = config_data.get("motion_pixel_count_thresholds", {})
        for r_key_new in app_state.range_definitions:
            app_state.motion_pixel_count_thresholds[r_key_new] = config_motion_count_thresholds.get(r_key_new, app_state.motion_pixel_count_thresholds[r_key_new])

        config_motion_frames_thresholds = config_data.get("motion_consecutive_frames_thresholds", {})
        for r_key_new in app_state.range_definitions:
            app_state.motion_consecutive_frames_thresholds[r_key_new] = config_motion_frames_thresholds.get(r_key_new, app_state.motion_consecutive_frames_thresholds[r_key_new])
        
        app_state.motion_hold_time_s = config_data.get("motion_hold_time_s", app_state.motion_hold_time_s)
        app_state.motion_allowed_gaps = config_data.get("motion_allowed_gaps", app_state.motion_allowed_gaps)

        loaded_threshold_list_1d = config_data.get("per_antenna_range_thresholds_1d")
        if loaded_threshold_list_1d and isinstance(loaded_threshold_list_1d, list) and len(loaded_threshold_list_1d) == NUM_RX_ANTENNAS:
            for i_ant in range(NUM_RX_ANTENNAS):
                config_ant_thresholds_1d = loaded_threshold_list_1d[i_ant] if i_ant < len(loaded_threshold_list_1d) and isinstance(loaded_threshold_list_1d[i_ant], dict) else {}
                for r_key_new in app_state.range_definitions:
                    app_state.per_antenna_range_thresholds_1d[i_ant][r_key_new] = config_ant_thresholds_1d.get(r_key_new, app_state.per_antenna_range_thresholds_1d[i_ant][r_key_new])
        else:
            print_once("Warning: 1D thresholds in config are missing, have incorrect number of antennas, or wrong format. Using defaults for all antennas.")

        loaded_threshold_list_2d = config_data.get("per_antenna_range_thresholds_2d")
        if loaded_threshold_list_2d and isinstance(loaded_threshold_list_2d, list) and len(loaded_threshold_list_2d) == NUM_RX_ANTENNAS:
            for i_ant in range(NUM_RX_ANTENNAS):
                config_ant_thresholds_2d = loaded_threshold_list_2d[i_ant] if i_ant < len(loaded_threshold_list_2d) and isinstance(loaded_threshold_list_2d[i_ant], dict) else {}
                for r_key_new in app_state.range_definitions:
                    app_state.per_antenna_range_thresholds_2d[i_ant][r_key_new] = config_ant_thresholds_2d.get(r_key_new, app_state.per_antenna_range_thresholds_2d[i_ant][r_key_new])
        else:
            print_once("Warning: 2D thresholds in config are missing, have incorrect number of antennas, or wrong format. Using defaults for all antennas.")


        app_state.home_assistant_enabled = config_data.get("home_assistant_enabled", app_state.home_assistant_enabled)
        app_state.home_assistant_url = config_data.get("home_assistant_url", app_state.home_assistant_url)
        app_state.home_assistant_token = config_data.get("home_assistant_token", app_state.home_assistant_token)
        app_state.home_assistant_light_entity_id = config_data.get("home_assistant_light_entity_id", app_state.home_assistant_light_entity_id)
        
        app_state.sustained_presence_enabled = config_data.get("sustained_presence_enabled", app_state.sustained_presence_enabled)
        app_state.sustained_min_hot_pixels_for_group = config_data.get("sustained_min_hot_pixels_for_group", app_state.sustained_min_hot_pixels_for_group)
        app_state.sustained_pixel_proximity_threshold = config_data.get("sustained_pixel_proximity_threshold", app_state.sustained_pixel_proximity_threshold)
        app_state.sustained_duration_threshold_s = config_data.get("sustained_duration_threshold_s", app_state.sustained_duration_threshold_s)
        app_state.sustained_magnification_factor = config_data.get("sustained_magnification_factor", app_state.sustained_magnification_factor)
        app_state.sustained_max_gap_frames = config_data.get("sustained_max_gap_frames", app_state.sustained_max_gap_frames)
        
        app_state.penalize_new_clusters_enabled = config_data.get("penalize_new_clusters_enabled", app_state.penalize_new_clusters_enabled)
        app_state.penalize_frames_duration = config_data.get("penalize_frames_duration", app_state.penalize_frames_duration)
        app_state.penalize_reduction_factor = config_data.get("penalize_reduction_factor", app_state.penalize_reduction_factor)

        print(f"Configuration loaded from {filepath}")
        if app_state.gui_is_active: update_gui_from_appstate(app_state)

    except FileNotFoundError: print(f"Configuration file {filepath} not found. Using default settings.")
    except json.JSONDecodeError: print(f"Error decoding JSON from {filepath}. Using default settings.")
    except Exception as e: print(f"Error loading configuration: {e}. Using default settings."); traceback.print_exc()

# <<< MODIFIED function to accept rgb_color >>>
def control_home_assistant_light(app_state: AppState, turn_on: bool, rgb_color: list = None):
    if not app_state.home_assistant_enabled: return
    if not app_state.home_assistant_url or not app_state.home_assistant_token or not app_state.home_assistant_light_entity_id:
        print_once("Home Assistant URL, Token, or Entity ID not configured.")
        return
    
    service = "turn_on" if turn_on else "turn_off"
    url = f"{app_state.home_assistant_url.rstrip('/')}/api/services/light/{service}"
    headers = {"Authorization": f"Bearer {app_state.home_assistant_token}", "Content-Type": "application/json"}
    data = {"entity_id": app_state.home_assistant_light_entity_id}

    if turn_on and rgb_color:
        data["rgb_color"] = rgb_color
        # Optional: Add brightness or other parameters if needed
        # data["brightness_pct"] = 100 

    try:
        response = requests.post(url, headers=headers, json=data, timeout=5)
        response.raise_for_status()
        action = "ON" if turn_on else "OFF"
        color_info = f"with color {rgb_color}" if turn_on and rgb_color else ""
        print_once(f"Light {app_state.home_assistant_light_entity_id} turned {action} {color_info} via Home Assistant.")
        app_state.light_is_on = turn_on
        if turn_on:
            app_state.current_light_color = rgb_color # <<< ADDED: Store the set color
        else:
            app_state.current_light_color = None # <<< ADDED: Clear color when off
    except requests.exceptions.RequestException as e: print_once(f"Error controlling Home Assistant light: {e}")
    except Exception as e: print_once(f"Unexpected error with Home Assistant: {e}")

def update_gui_from_appstate(app_state: AppState):
    if not app_state.gui_is_active: return

    if app_state.gui_auto_cal_enabled_var: app_state.gui_auto_cal_enabled_var.set(app_state.auto_calibration_enabled)
    if app_state.gui_auto_cal_interval_var: app_state.gui_auto_cal_interval_var.set(f"{app_state.auto_calibration_interval_s:.1f}")

    for r_key, gui_vars_for_range in app_state.gui_motion_vars_by_range.items():
        if "mag_var" in gui_vars_for_range and gui_vars_for_range["mag_var"]:
            default_mag = app_state.motion_pixel_magnitude_thresholds.get("0.0-0.5m", 0.01) 
            gui_vars_for_range["mag_var"].set(f"{app_state.motion_pixel_magnitude_thresholds.get(r_key, default_mag):.4f}")
        if "count_var" in gui_vars_for_range and gui_vars_for_range["count_var"]:
            default_count = app_state.motion_pixel_count_thresholds.get("0.0-0.5m", 10) 
            gui_vars_for_range["count_var"].set(f"{app_state.motion_pixel_count_thresholds.get(r_key, default_count):.0f}")
        if "frames_var" in gui_vars_for_range and gui_vars_for_range["frames_var"]:
            default_frames = app_state.motion_consecutive_frames_thresholds.get("0.0-0.5m", 2) 
            gui_vars_for_range["frames_var"].set(f"{app_state.motion_consecutive_frames_thresholds.get(r_key, default_frames):.0f}")
    
    if app_state.gui_motion_generic_vars.get("hold_time_var"):
        app_state.gui_motion_generic_vars["hold_time_var"].set(f"{app_state.motion_hold_time_s:.1f}")
    if app_state.gui_motion_generic_vars.get("allowed_gaps_var"):
        app_state.gui_motion_generic_vars["allowed_gaps_var"].set(f"{app_state.motion_allowed_gaps:.0f}")

    for i_ant_gui in range(NUM_RX_ANTENNAS):
        default_1d_thresh_val = app_state.per_antenna_range_thresholds_1d[i_ant_gui].get("0.0-0.5m", 0.0)
        default_2d_thresh_val = app_state.per_antenna_range_thresholds_2d[i_ant_gui].get("0.0-0.5m", 0.0)

        for range_key, var_obj in app_state.gui_1d_threshold_vars[i_ant_gui].items():
            if var_obj: var_obj.set(f"{app_state.per_antenna_range_thresholds_1d[i_ant_gui].get(range_key, default_1d_thresh_val):.3f}")
        for range_key, var_obj in app_state.gui_2d_threshold_vars[i_ant_gui].items():
            if var_obj: var_obj.set(f"{app_state.per_antenna_range_thresholds_2d[i_ant_gui].get(range_key, default_2d_thresh_val):.4f}")
    
    if app_state.gui_ha_vars.get("enabled_var"): app_state.gui_ha_vars["enabled_var"].set(app_state.home_assistant_enabled)
    if app_state.gui_ha_vars.get("url_var"): app_state.gui_ha_vars["url_var"].set(app_state.home_assistant_url)
    if app_state.gui_ha_vars.get("token_var"): app_state.gui_ha_vars["token_var"].set(app_state.home_assistant_token)
    if app_state.gui_ha_vars.get("entity_id_var"): app_state.gui_ha_vars["entity_id_var"].set(app_state.home_assistant_light_entity_id)

    if app_state.gui_sustained_vars.get("enabled_var"): app_state.gui_sustained_vars["enabled_var"].set(app_state.sustained_presence_enabled)
    sustained_fmt_map = {"sustained_min_hot_pixels_for_group": "%.0f", "sustained_pixel_proximity_threshold": "%.0f", "sustained_duration_threshold_s": "%.1f", "sustained_magnification_factor": "%.2f", "sustained_max_gap_frames": "%.0f"}
    for attr, var_obj in app_state.gui_sustained_vars.items():
        if var_obj and attr != "enabled_var": value = getattr(app_state, attr); fmt = sustained_fmt_map.get(attr, "%.1f"); var_obj.set(f"{value:{fmt.replace('%','')}}")

    if app_state.gui_penalize_vars.get("enabled_var"): app_state.gui_penalize_vars["enabled_var"].set(app_state.penalize_new_clusters_enabled)
    penalize_fmt_map = {"penalize_frames_duration": "%.0f", "penalize_reduction_factor": "%.2f"}
    for attr, var_obj in app_state.gui_penalize_vars.items():
        if var_obj and attr != "enabled_var": value = getattr(app_state, attr); fmt = penalize_fmt_map.get(attr, "%.2f"); var_obj.set(f"{value:{fmt.replace('%','')}}")
    print("GUI updated from AppState.")

def setup_threshold_gui(app_state_obj: AppState):
    root = tk.Tk()
    app_state_obj.gui_root = root
    root.title("Radar Controls")
    num_antennas_for_gui = NUM_RX_ANTENNAS
    gui_width = 750 

    num_ranges = len(app_state_obj.range_definitions)
    motion_per_range_section_height = num_ranges * 30 + 40 
    total_height = 70 + motion_per_range_section_height + 70 + 200 + 120 + 150 + 40 + 30 + (num_ranges * 25) 
    root.geometry(f"{gui_width}x{min(total_height, 950)}")


    main_frame = ttk.Frame(root)
    main_frame.pack(expand=True, fill='both', padx=5, pady=5)

    def make_on_change_cb(target_dict, key_name, s_var, s_fmt, s_params=None, is_float=True):
        def on_change_cb_internal(*args):
            try:
                val = float(s_var.get()) if is_float else int(s_var.get())
                if s_params:
                    min_val, max_val = s_params[0], s_params[1]
                    if val < min_val or val > max_val:
                        current_val = target_dict.get(key_name, (min_val + max_val) / 2 if is_float else int((min_val + max_val) / 2))
                        s_var.set(f"{current_val:{s_fmt.replace('%','')}}")
                        return
                target_dict[key_name] = val
            except ValueError:
                current_val = target_dict.get(key_name, 0.0 if is_float else 0)
                s_var.set(f"{current_val:{s_fmt.replace('%','')}}")
        return on_change_cb_internal

    def make_on_change_attr_cb(attr_name, s_var, s_fmt, s_params=None, is_float=True):
        def on_change_cb_attr(*args):
            try:
                val = float(s_var.get()) if is_float else int(s_var.get())
                if s_params:
                    min_val, max_val = s_params[0], s_params[1]
                    if val < min_val or val > max_val:
                        s_var.set(f"{getattr(app_state_obj, attr_name):{s_fmt.replace('%','')}}")
                        return
                setattr(app_state_obj, attr_name, val)
            except ValueError:
                 s_var.set(f"{getattr(app_state_obj, attr_name):{s_fmt.replace('%','')}}")
        return on_change_cb_attr

    top_controls_frame = ttk.Frame(main_frame)
    top_controls_frame.pack(pady=5, padx=10, fill='x', anchor='n')
    app_state_obj.gui_auto_cal_enabled_var = tk.BooleanVar(value=app_state_obj.auto_calibration_enabled)
    def toggle_auto_calibration():
        app_state_obj.auto_calibration_enabled = app_state_obj.gui_auto_cal_enabled_var.get()
        if app_state_obj.auto_calibration_enabled: app_state_obj.last_calibration_time = time.time()
    auto_cal_check = ttk.Checkbutton(top_controls_frame, text="Enable Auto-Cal", variable=app_state_obj.gui_auto_cal_enabled_var, command=toggle_auto_calibration)
    auto_cal_check.pack(side=tk.LEFT, padx=(0,5))
    ttk.Label(top_controls_frame, text="Interval(s):").pack(side=tk.LEFT)
    app_state_obj.gui_auto_cal_interval_var = tk.StringVar(value=f"{app_state_obj.auto_calibration_interval_s:.1f}")
    app_state_obj.gui_auto_cal_interval_var.trace_add("write", make_on_change_attr_cb("auto_calibration_interval_s", app_state_obj.gui_auto_cal_interval_var, "%.1f", (0.5, 60.0)))
    auto_cal_spinbox = ttk.Spinbox(top_controls_frame, from_=0.5, to=60.0, increment=0.5, width=5, textvariable=app_state_obj.gui_auto_cal_interval_var, format="%.1f")
    auto_cal_spinbox.pack(side=tk.LEFT, padx=5)

    motion_lf = ttk.LabelFrame(main_frame, text="Motion Detection Settings (Per Range, Ant 0 R-D)")
    motion_lf.pack(pady=10, padx=10, fill='x', anchor='n')
    
    header_f = ttk.Frame(motion_lf); header_f.pack(fill='x')
    ttk.Label(header_f, text="Range Zone", width=12).pack(side=tk.LEFT, padx=2)
    ttk.Label(header_f, text="Mag Thresh", width=12).pack(side=tk.LEFT, padx=2)
    ttk.Label(header_f, text="Count Thresh", width=12).pack(side=tk.LEFT, padx=2)
    ttk.Label(header_f, text="Frames Thresh", width=12).pack(side=tk.LEFT, padx=2)

    app_state_obj.gui_motion_vars_by_range = {k: {} for k in app_state_obj.range_definitions}
    for r_key in app_state_obj.range_definitions: 
        range_f = ttk.Frame(motion_lf); range_f.pack(fill='x', pady=1)
        ttk.Label(range_f, text=r_key, width=12).pack(side=tk.LEFT, padx=2)

        default_mag = app_state_obj.motion_pixel_magnitude_thresholds.get(r_key, 0.01)
        mag_var = tk.StringVar(value=f"{default_mag:.4f}")
        app_state_obj.gui_motion_vars_by_range[r_key]["mag_var"] = mag_var
        mag_var.trace_add("write", make_on_change_cb(app_state_obj.motion_pixel_magnitude_thresholds, r_key, mag_var, "%.4f", (0.0001, 0.5)))
        ttk.Spinbox(range_f, from_=0.0001, to=0.5, increment=0.0001, width=10, format="%.4f", textvariable=mag_var).pack(side=tk.LEFT, padx=2)

        default_count = app_state_obj.motion_pixel_count_thresholds.get(r_key, 10)
        count_var = tk.StringVar(value=f"{default_count:.0f}")
        app_state_obj.gui_motion_vars_by_range[r_key]["count_var"] = count_var
        count_var.trace_add("write", make_on_change_cb(app_state_obj.motion_pixel_count_thresholds, r_key, count_var, "%.0f", (1, 200), is_float=False))
        ttk.Spinbox(range_f, from_=1, to=200, increment=1, width=10, format="%.0f", textvariable=count_var).pack(side=tk.LEFT, padx=2)

        default_frames = app_state_obj.motion_consecutive_frames_thresholds.get(r_key, 2)
        frames_var = tk.StringVar(value=f"{default_frames:.0f}")
        app_state_obj.gui_motion_vars_by_range[r_key]["frames_var"] = frames_var
        frames_var.trace_add("write", make_on_change_cb(app_state_obj.motion_consecutive_frames_thresholds, r_key, frames_var, "%.0f", (1, 10), is_float=False))
        ttk.Spinbox(range_f, from_=1, to=10, increment=1, width=10, format="%.0f", textvariable=frames_var).pack(side=tk.LEFT, padx=2)

    motion_generic_lf = ttk.LabelFrame(main_frame, text="Global Motion Logic")
    motion_generic_lf.pack(pady=5, padx=10, fill='x', anchor='n')
    app_state_obj.gui_motion_generic_vars = {}
    
    g_row1 = ttk.Frame(motion_generic_lf); g_row1.pack(fill='x')
    ttk.Label(g_row1, text="Overall Hold Time (s):", width=22).pack(side=tk.LEFT)
    hold_time_var = tk.StringVar(value=f"{app_state_obj.motion_hold_time_s:.1f}")
    app_state_obj.gui_motion_generic_vars["hold_time_var"] = hold_time_var
    hold_time_var.trace_add("write", make_on_change_attr_cb("motion_hold_time_s", hold_time_var, "%.1f", (0.1, 30.0)))
    ttk.Spinbox(g_row1, from_=0.1, to=30.0, increment=0.1, width=8, format="%.1f", textvariable=hold_time_var).pack(side=tk.LEFT, padx=5)

    ttk.Label(g_row1, text="Allowed Frame Gaps (Overall):", width=25).pack(side=tk.LEFT, padx=(10,0))
    allowed_gaps_var = tk.StringVar(value=f"{app_state_obj.motion_allowed_gaps:.0f}")
    app_state_obj.gui_motion_generic_vars["allowed_gaps_var"] = allowed_gaps_var
    allowed_gaps_var.trace_add("write", make_on_change_attr_cb("motion_allowed_gaps", allowed_gaps_var, "%.0f", (0, 10), is_float=False))
    ttk.Spinbox(g_row1, from_=0, to=10, increment=1, width=8, format="%.0f", textvariable=allowed_gaps_var).pack(side=tk.LEFT, padx=5)

    sustained_lf = ttk.LabelFrame(main_frame, text="Sustained Presence Magnification (Ant 0 R-D)")
    sustained_lf.pack(pady=10, padx=10, fill='x', anchor='n')
    app_state_obj.gui_sustained_vars = {}
    sustained_enabled_var = tk.BooleanVar(value=app_state_obj.sustained_presence_enabled)
    def toggle_sustained_enabled(): app_state_obj.sustained_presence_enabled = sustained_enabled_var.get()
    sustained_check = ttk.Checkbutton(sustained_lf, text="Enable Sustained Detection", variable=sustained_enabled_var, command=toggle_sustained_enabled)
    sustained_check.grid(row=0, column=0, columnspan=2, sticky='w', pady=(5,2))
    app_state_obj.gui_sustained_vars["enabled_var"] = sustained_enabled_var
    sustained_settings_layout = [
        ("Min Pixels for Group:", "sustained_min_hot_pixels_for_group", (1, 50, 1), "%.0f", False),
        ("Pixel Proximity (bins):", "sustained_pixel_proximity_threshold", (1, 10, 1), "%.0f", False),
        ("Sustained Duration (s):", "sustained_duration_threshold_s", (0.5, 10.0, 0.1), "%.1f", True),
        ("Magnification Factor:", "sustained_magnification_factor", (1.0, 5.0, 0.1), "%.2f", True),
        ("Max Gap (frames):", "sustained_max_gap_frames", (0, 20, 1), "%.0f", False)
    ]
    for i, (lbl_txt, attr, spin_p, fmt, is_f) in enumerate(sustained_settings_layout):
        row_idx_sustained = (i // 2) + 1; col_idx_sustained = i % 2
        row_f_sustained = ttk.Frame(sustained_lf); row_f_sustained.grid(row=row_idx_sustained, column=col_idx_sustained, sticky='ew', pady=1, padx=2)
        ttk.Label(row_f_sustained, text=lbl_txt, width=22).pack(side=tk.LEFT)
        var_sustained = tk.StringVar(value=f"{getattr(app_state_obj, attr):{fmt.replace('%','')}}")
        app_state_obj.gui_sustained_vars[attr] = var_sustained
        var_sustained.trace_add("write", make_on_change_attr_cb(attr, var_sustained, fmt, spin_p, is_float=is_f))
        ttk.Spinbox(row_f_sustained, from_=spin_p[0],to=spin_p[1],increment=spin_p[2],width=8,format=fmt,textvariable=var_sustained).pack(side=tk.LEFT,padx=5)
    for col_idx in range(2): sustained_lf.grid_columnconfigure(col_idx, weight=1)

    penalize_lf = ttk.LabelFrame(main_frame, text="New Cluster Penalization (Ant 0 R-D)")
    penalize_lf.pack(pady=10, padx=10, fill='x', anchor='n')
    app_state_obj.gui_penalize_vars = {}
    penalize_enabled_var = tk.BooleanVar(value=app_state_obj.penalize_new_clusters_enabled)
    def toggle_penalize_enabled(): app_state_obj.penalize_new_clusters_enabled = penalize_enabled_var.get()
    penalize_check = ttk.Checkbutton(penalize_lf, text="Enable New Cluster Penalization", variable=penalize_enabled_var, command=toggle_penalize_enabled)
    penalize_check.grid(row=0, column=0, columnspan=2, sticky='w', pady=(5,2))
    app_state_obj.gui_penalize_vars["enabled_var"] = penalize_enabled_var
    penalize_settings_layout = [
        ("Penalty Duration (frames):", "penalize_frames_duration", (1, 100, 1), "%.0f", False),
        ("Reduction Factor (0.0-1.0):", "penalize_reduction_factor", (0.0, 1.0, 0.01), "%.2f", True),
    ]
    for i, (lbl_txt, attr, spin_p, fmt, is_f) in enumerate(penalize_settings_layout):
        row_idx_penalize = (i // 2) + 1; col_idx_penalize = i % 2
        row_f_penalize = ttk.Frame(penalize_lf); row_f_penalize.grid(row=row_idx_penalize, column=col_idx_penalize, sticky='ew', pady=1, padx=2)
        ttk.Label(row_f_penalize, text=lbl_txt, width=22).pack(side=tk.LEFT)
        var_penalize = tk.StringVar(value=f"{getattr(app_state_obj, attr):{fmt.replace('%','')}}")
        app_state_obj.gui_penalize_vars[attr] = var_penalize
        var_penalize.trace_add("write", make_on_change_attr_cb(attr, var_penalize, fmt, spin_p, is_float=is_f))
        ttk.Spinbox(row_f_penalize, from_=spin_p[0], to=spin_p[1], increment=spin_p[2], width=8, format=fmt, textvariable=var_penalize).pack(side=tk.LEFT, padx=5)
    for col_idx in range(2): penalize_lf.grid_columnconfigure(col_idx, weight=1)

    ha_lf = ttk.LabelFrame(main_frame, text="Home Assistant Control")
    ha_lf.pack(pady=10, padx=10, fill='x', anchor='n')
    app_state_obj.gui_ha_vars = {}
    ha_enabled_var = tk.BooleanVar(value=app_state_obj.home_assistant_enabled)
    def toggle_ha_enabled(): app_state_obj.home_assistant_enabled = ha_enabled_var.get()
    ha_check = ttk.Checkbutton(ha_lf, text="Enable Home Assistant", variable=ha_enabled_var, command=toggle_ha_enabled)
    ha_check.grid(row=0, column=0, columnspan=2, sticky='w', pady=2)
    app_state_obj.gui_ha_vars["enabled_var"] = ha_enabled_var
    ha_entries = [("HA URL:", "home_assistant_url", False), ("HA Token:", "home_assistant_token", True), ("Light Entity ID:", "home_assistant_light_entity_id", False)]
    for r, (lbl, attr, is_secret) in enumerate(ha_entries):
        ttk.Label(ha_lf, text=lbl).grid(row=r+1, column=0, sticky='w', padx=2)
        var = tk.StringVar(value=getattr(app_state_obj, attr))
        def make_ha_cb(a, v): return lambda *args: setattr(app_state_obj, a, v.get())
        var.trace_add("write", make_ha_cb(attr, var))
        entry = ttk.Entry(ha_lf, textvariable=var, width=40, show="*" if is_secret else None)
        entry.grid(row=r+1, column=1, sticky='ew', padx=2)
        app_state_obj.gui_ha_vars[f"{attr}_var"] = var 
    ha_lf.grid_columnconfigure(1, weight=1)

    notebook_frame = ttk.Frame(main_frame)
    notebook_frame.pack(expand=True, fill='both', pady=5, padx=5)
    notebook = ttk.Notebook(notebook_frame)
    notebook.pack(expand=True, fill='both')
    def on_thresh_spinbox_change(var_obj, target_dict_list, ant_idx_cb, r_key_cb, format_str_cb, spin_from_val):
        try:
            val = float(var_obj.get())
            if val >= spin_from_val: target_dict_list[ant_idx_cb][r_key_cb] = val
            else: var_obj.set(f"{target_dict_list[ant_idx_cb].get(r_key_cb, spin_from_val):{format_str_cb.replace('%','')}}")
        except ValueError: var_obj.set(f"{target_dict_list[ant_idx_cb].get(r_key_cb, spin_from_val):{format_str_cb.replace('%','')}}")

    for i_ant_gui in range(num_antennas_for_gui):
        ant_tab = ttk.Frame(notebook); notebook.add(ant_tab, text=f"Ant {i_ant_gui} Thresh.")
        tab_col1 = ttk.Frame(ant_tab); tab_col1.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5, anchor='n', expand=True)
        tab_col2 = ttk.Frame(ant_tab); tab_col2.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5, anchor='n', expand=True)
        
        app_state_obj.gui_1d_threshold_vars[i_ant_gui] = {} 
        app_state_obj.gui_2d_threshold_vars[i_ant_gui] = {} 

        ttk.Label(tab_col1, text="1D Profile Ranges:").pack(pady=(5,2), anchor='w')
        spin_from_1d, increment_1d, fmt_1d = 0.0, 0.001, "%.3f"
        for range_key, _ in app_state_obj.range_definitions.items(): 
            default_val = app_state_obj.per_antenna_range_thresholds_1d[i_ant_gui].get(range_key, 0.01)
            item_f = ttk.Frame(tab_col1); item_f.pack(fill='x', pady=1)
            ttk.Label(item_f, text=f"{range_key}:", width=10).pack(side=tk.LEFT)
            var = tk.StringVar(value=f"{default_val:{fmt_1d.replace('%','')}}")
            app_state_obj.gui_1d_threshold_vars[i_ant_gui][range_key] = var
            cb = lambda *a,v=var,d=app_state_obj.per_antenna_range_thresholds_1d,i=i_ant_gui,k=range_key,f=fmt_1d,sf=spin_from_1d: on_thresh_spinbox_change(v,d,i,k,f,sf)
            var.trace_add("write", cb)
            ttk.Spinbox(item_f,from_=spin_from_1d,increment=increment_1d,width=10,format=fmt_1d,textvariable=var, to=1.0).pack(side=tk.LEFT,padx=2) 
        
        ttk.Label(tab_col2, text="2D Map Ranges (R-C/R-D):").pack(pady=(5,2), anchor='w')
        is_ant0_2d = (i_ant_gui == 0 and "9200.0" in str(app_state_obj.per_antenna_range_thresholds_2d[i_ant_gui].get("0.0-0.5m"))) 
        spin_from_2d = 0.0
        increment_2d = 100.0 if is_ant0_2d else 0.0001
        fmt_2d = "%.1f" if is_ant0_2d else "%.4f"
        spin_to_2d = 10000.0 if is_ant0_2d else 1.0

        for range_key, _ in app_state_obj.range_definitions.items(): 
            default_val = app_state_obj.per_antenna_range_thresholds_2d[i_ant_gui].get(range_key, 0.001)
            item_f2d = ttk.Frame(tab_col2); item_f2d.pack(fill='x', pady=1)
            ttk.Label(item_f2d, text=f"{range_key}:", width=10).pack(side=tk.LEFT)
            var2d = tk.StringVar(value=f"{default_val:{fmt_2d.replace('%','')}}")
            app_state_obj.gui_2d_threshold_vars[i_ant_gui][range_key] = var2d
            cb2 = lambda *a,v=var2d,d=app_state_obj.per_antenna_range_thresholds_2d,i=i_ant_gui,k=range_key,f=fmt_2d,sf=spin_from_2d: on_thresh_spinbox_change(v,d,i,k,f,sf)
            var2d.trace_add("write", cb2)
            ttk.Spinbox(item_f2d,from_=spin_from_2d,increment=increment_2d,width=10,format=fmt_2d,textvariable=var2d, to=spin_to_2d).pack(side=tk.LEFT,padx=2)
    
    config_button_frame = ttk.Frame(main_frame)
    config_button_frame.pack(pady=10, padx=10, fill='x', side=tk.BOTTOM)
    def do_save_config(): save_configuration(app_state_obj, CONFIG_FILE_PATH)
    def do_load_config(): load_configuration(app_state_obj, CONFIG_FILE_PATH); update_gui_from_appstate(app_state_obj)
    save_button = ttk.Button(config_button_frame, text="Save Configuration", command=do_save_config); save_button.pack(side=tk.LEFT, padx=5)
    load_button = ttk.Button(config_button_frame, text="Load Configuration", command=do_load_config); load_button.pack(side=tk.LEFT, padx=5)

    root.protocol("WM_DELETE_WINDOW", lambda: setattr(app_state_obj, 'gui_is_active', False))
    app_state_obj.gui_is_active = True
    return root

class RadarConfig:
    def __init__(self, num_chirps, num_samples, start_freq, end_freq, proc_framerate_hz, sample_rate_hz, chirp_rep_time_s):
        self.num_chirps_per_frame = num_chirps; self.num_samples_per_chirp = num_samples
        self.start_frequency_Hz = start_freq; self.end_frequency_Hz = end_freq
        self.frame_repetition_time_s = 1.0 / proc_framerate_hz if proc_framerate_hz > 0 else 0.01
        self.sample_rate_Hz = sample_rate_hz; self.rx_mask = (1 << NUM_RX_ANTENNAS) - 1; self.tx_mask = 1
        self.tx_power_level = 31; self.if_gain_dB = 33; self.chirp_repetition_time_s = chirp_rep_time_s
        self.center_frequency_Hz = (start_freq + end_freq) / 2.0 if (start_freq + end_freq) > 0 else 0
        self.wavelength_m = constants.c / self.center_frequency_Hz if self.center_frequency_Hz > 0 else 0
        if self.center_frequency_Hz == 0: print_once("Warning: Center frequency is zero in RadarConfig.")
        if self.wavelength_m == 0 and self.center_frequency_Hz > 0 : print_once("Warning: Wavelength is zero in RadarConfig despite non-zero center frequency.")

def fft_spectrum(data_matrix, window_vector):
    if data_matrix.shape[1] == 0 : return np.array([[]]*data_matrix.shape[0])
    if window_vector.shape[1] != data_matrix.shape[1]:
        print_once(f"Warning: FFT window size {window_vector.shape} mismatch with data {data_matrix.shape}. Using unwindowed.")
        windowed_data = data_matrix
    else: windowed_data = data_matrix * window_vector
    fft_size = data_matrix.shape[1] * 2
    complex_fft_result = np.fft.fft(windowed_data, n=fft_size, axis=1)
    return complex_fft_result[:, :data_matrix.shape[1]]

class DistanceFFT_Algo:
    def __init__(self, config: RadarConfig):
        self._numchirps=config.num_chirps_per_frame; self.chirpsamples=config.num_samples_per_chirp
        self._range_window=windows.blackmanharris(self.chirpsamples).reshape(1,self.chirpsamples) if self.chirpsamples > 0 else np.array([[]])
        bandwidth_hz = abs(config.end_frequency_Hz-config.start_frequency_Hz)
        fft_s_bin = self.chirpsamples*2 if self.chirpsamples > 0 else 2
        self._range_bin_length = (constants.c)/(2*bandwidth_hz*(fft_s_bin/ (self.chirpsamples if self.chirpsamples > 0 else 1) )) if bandwidth_hz>0 else 0
        if self._range_bin_length==0: print_once("Warning: Range bin length is 0. Check radar config.")
        self._config=config
        if self._numchirps > 0:
            self._doppler_window=windows.hann(self._numchirps).reshape(self._numchirps,1)
            if config.wavelength_m > 0 and config.chirp_repetition_time_s > 0:
                d_freqs_s=np.fft.fftshift(np.fft.fftfreq(self._numchirps,d=config.chirp_repetition_time_s))
                self._velocity_axis_mps = d_freqs_s * config.wavelength_m / 2.0
                self.max_unambiguous_velocity_mps = (1.0/(2.0*config.chirp_repetition_time_s)*config.wavelength_m)/2.0
                print(f"Max unambiguous velocity: +/-{self.max_unambiguous_velocity_mps:.2f} m/s")
            else: self._velocity_axis_mps=np.zeros(self._numchirps);self.max_unambiguous_velocity_mps=0;print_once("Warn: Velocity axis calc failed.")
        else: self._doppler_window=np.array([]); self._velocity_axis_mps=np.array([]); self.max_unambiguous_velocity_mps=0; print_once("Warn: Doppler processing disabled.")

    def _compute_range_doppler_map(self, range_fft_complex_data):
        if self._numchirps==0 or range_fft_complex_data.ndim != 2 or range_fft_complex_data.shape[0]!=self._numchirps or range_fft_complex_data.shape[1] == 0:
            num_valid_chirps = self._numchirps if self._numchirps > 0 else 1
            num_valid_samples = self.chirpsamples if self.chirpsamples > 0 else 1
            if range_fft_complex_data.ndim == 2 and range_fft_complex_data.shape[1] > 0: num_valid_samples = range_fft_complex_data.shape[1]
            return np.zeros((num_valid_chirps, num_valid_samples))
        windowed_data_for_doppler = range_fft_complex_data * self._doppler_window
        range_doppler_fft_complex = np.fft.fft(windowed_data_for_doppler, axis=0)
        return np.abs(np.fft.fftshift(range_doppler_fft_complex, axes=0))

    def compute_distance_and_profile(self, data):
        if not isinstance(data, np.ndarray) or data.ndim != 2 or data.shape[0] == 0 or data.shape[1] == 0 or self.chirpsamples == 0:
            s = self.chirpsamples if self.chirpsamples > 0 else 1; c = self._numchirps if self._numchirps > 0 else 1
            print_once(f"Warning: Invalid data for FFT. Data shape: {data.shape if isinstance(data, np.ndarray) else 'Not ndarray'}")
            return 0, np.zeros(s), np.zeros((c,s)), np.zeros((c,s)) 
        range_fft_comp = fft_spectrum(data, self._range_window)
        fft_spec_abs_pc = np.abs(range_fft_comp)
        data_plot_1d = np.divide(fft_spec_abs_pc.sum(axis=0),self._numchirps) if self._numchirps > 0 and fft_spec_abs_pc.shape[0] == self._numchirps else (fft_spec_abs_pc[0,:] if fft_spec_abs_pc.ndim > 1 and fft_spec_abs_pc.shape[0]>0 else (fft_spec_abs_pc if fft_spec_abs_pc.ndim == 1 else np.zeros(self.chirpsamples if self.chirpsamples > 0 else 1)))
        if data_plot_1d.ndim == 0: data_plot_1d = np.array([data_plot_1d]) if data_plot_1d.size == 1 else np.zeros(self.chirpsamples if self.chirpsamples > 0 else 1)
        skip = BINS_TO_SKIP_FOR_PEAK_DETECTION
        dist = ( (np.argmax(data_plot_1d[skip:])+skip)*self._range_bin_length ) if data_plot_1d.size>skip and self._range_bin_length is not None else 0
        rd_map_abs = self._compute_range_doppler_map(range_fft_comp)
        return dist, data_plot_1d, fft_spec_abs_pc, rd_map_abs

class Draw:
    def __init__(self, config: RadarConfig, max_range_m_calc, num_ant, range_bin_length, velocity_axis_mps, app_state: AppState):
        self._num_ant = num_ant; self.config = config; self.chirpsamples = config.num_samples_per_chirp
        self.app_state = app_state
        self._line_plots_orig, self._image_plots_rc_orig, self._colorbars_rc_orig, self._range_doppler_plots_orig, self._range_doppler_colorbars_orig = [],[],[],[],[]
        self._line_plots_filt, self._image_plots_rc_filt, self._colorbars_rc_filt, self._range_doppler_plots_filt, self._range_doppler_colorbars_filt = [],[],[],[],[]
        num_cols_plot = self._num_ant*2; base_fig_width_per_pair = 7; fig_width = min(base_fig_width_per_pair * self._num_ant, 35); fig_height = 12
        self._fig,self._axs = plt.subplots(nrows=3,ncols=num_cols_plot,figsize=(fig_width,fig_height),squeeze=False, constrained_layout=False)
        self._fig.canvas.manager.set_window_title(f"Radar FFT ({NUM_SAMPLES_PER_CHIRP}s/{NUM_CHIRPS_PER_FRAME}c) | Press '{CALIBRATION_KEY}' to cal | {self._num_ant} RX")
        self.bins_to_ignore_for_1d_plot_ylim = BINS_TO_IGNORE_FOR_PLOT_YLIM
        self._range_bin_length = range_bin_length if range_bin_length is not None else 0.0
        self._velocity_axis_mps = velocity_axis_mps if velocity_axis_mps is not None else np.array([0])
        self._dist_points = np.array([i*self._range_bin_length for i in range(self.chirpsamples)]) if self.chirpsamples>0 and self._range_bin_length > 0 else np.array([0.0])
        self._chirp_indices = np.arange(self.config.num_chirps_per_frame if self.config.num_chirps_per_frame > 0 else 1)
        self._fig.canvas.mpl_connect('close_event',self.close); self._fig.canvas.mpl_connect('key_press_event',self._on_key_press)
        self._is_window_open=True; plt.ion(); self.motion_status_text_ant0 = None

    def _on_key_press(self, event): self.app_state.calibration_requested = event.key == CALIBRATION_KEY
    def _get_1d_plot_ylimits(self, data_all_antennas_1d_profiles, is_filtered=False):
        min_val,max_val = float('inf'),float('-inf'); has_valid_data = False
        for data_one_ant in data_all_antennas_1d_profiles:
            if not isinstance(data_one_ant,np.ndarray) or data_one_ant.size==0: continue
            has_valid_data = True; bins_ign = self.bins_to_ignore_for_1d_plot_ylim
            data_calc = data_one_ant[bins_ign:] if not is_filtered and data_one_ant.size>bins_ign else data_one_ant
            if data_calc.size>0: cur_max=np.max(data_calc);cur_min=np.min(data_calc);max_val=max(max_val,cur_max);min_val=min(min_val,cur_min)
        if not has_valid_data or max_val==float('-inf'):max_val= (0.1 if is_filtered else 1.0)
        if not has_valid_data or min_val==float('inf'):min_val= (-0.01 if is_filtered else 0.0)
        if max_val<=min_val:max_val=min_val+(0.01 if is_filtered else 0.1)
        if is_filtered:min_val=min(min_val,-0.01)
        return min_val,max_val*(1.02 if is_filtered else 1.05)

    def _draw_first_time(self, data_1d_orig, data_2d_rc_orig, data_rd_orig, data_1d_filt, data_2d_rc_filt, data_rd_filt):
        min_y_1do,max_y_1do=self._get_1d_plot_ylimits(data_1d_orig); min_y_1df,max_y_1df=self._get_1d_plot_ylimits(data_1d_filt,True)
        dist_s,dist_e=(self._dist_points[0],self._dist_points[-1]) if len(self._dist_points)>1 else (0,self._dist_points[0] + 0.1 if len(self._dist_points)==1 else 1)
        vel_s,vel_e=(self._velocity_axis_mps[0],self._velocity_axis_mps[-1]) if len(self._velocity_axis_mps)>1 else (-1,1)
        chirp_s, chirp_e = (self._chirp_indices[0]-0.5, self._chirp_indices[-1]+0.5) if len(self._chirp_indices) > 0 else (-0.5, 0.5)
        for i_ant in range(self._num_ant):
            co,cf=i_ant*2,i_ant*2+1
            d1o_ant=data_1d_orig[i_ant] if i_ant<len(data_1d_orig) else np.zeros_like(self._dist_points);d1f_ant=data_1d_filt[i_ant] if i_ant<len(data_1d_filt) else np.zeros_like(self._dist_points)
            d2rco_ant=data_2d_rc_orig[i_ant] if i_ant<len(data_2d_rc_orig) else np.zeros((len(self._chirp_indices),len(self._dist_points))); d2rcf_ant=data_2d_rc_filt[i_ant] if i_ant<len(data_2d_rc_filt) else np.zeros((len(self._chirp_indices),len(self._dist_points)))
            drdo_ant=data_rd_orig[i_ant] if i_ant<len(data_rd_orig) else np.zeros((len(self._velocity_axis_mps),len(self._dist_points))); drdf_ant=data_rd_filt[i_ant] if i_ant<len(data_rd_filt) else np.zeros((len(self._velocity_axis_mps),len(self._dist_points)))
            dp1o=self._dist_points[:len(d1o_ant)]; dp1f=self._dist_points[:len(d1f_ant)]
            if dp1o.size==0: dp1o=np.array([0.0])
            if d1o_ant.size==0: d1o_ant=np.array([0.0])
            if dp1f.size==0: dp1f=np.array([0.0])
            if d1f_ant.size==0: d1f_ant=np.array([0.0])
            lo,=self._axs[0,co].plot(dp1o,d1o_ant);self._axs[0,co].set_ylim(min_y_1do,max_y_1do);self._line_plots_orig.append(lo);self._axs[0,co].set_xlabel("Dist(m)");self._axs[0,co].set_ylabel("FFT Mag");self._axs[0,co].set_title(f"Ant{i_ant} Orig Rng")
            lf,=self._axs[0,cf].plot(dp1f,d1f_ant);self._axs[0,cf].set_ylim(min_y_1df,max_y_1df);self._line_plots_filt.append(lf);self._axs[0,cf].set_xlabel("Dist(m)");self._axs[0,cf].set_ylabel("Filt FFT Mag");self._axs[0,cf].set_title(f"Ant{i_ant} Filt Rng")
            d2o=20*np.log10(d2rco_ant+1e-9);vmo,vxo=np.percentile(d2o,[5,95]) if d2o.size>0 else(-60,0);vxo=max(vxo,vmo+1);imo=self._axs[1,co].imshow(d2o,aspect='auto',cmap='viridis',origin='lower',vmin=vmo,vmax=vxo,extent=[dist_s,dist_e,chirp_s,chirp_e]);self._image_plots_rc_orig.append(imo);cbo=self._fig.colorbar(imo,ax=self._axs[1,co],label='Mag(dB)');self._colorbars_rc_orig.append(cbo);self._axs[1,co].set_xlabel("Dist(m)");self._axs[1,co].set_ylabel("ChirpIdx");self._axs[1,co].set_title(f"Ant{i_ant} Orig Rng/Chirp")
            d2f=20*np.log10(d2rcf_ant+FILTERING_FLOOR_2D_MAG_LINEAR);vmf,vxf=np.percentile(d2f,[5,98]) if d2f.size>0 else(-70,-10);vxf=max(vxf, vmf+10);imf=self._axs[1,cf].imshow(d2f,aspect='auto',cmap='viridis',origin='lower',vmin=vmf,vmax=vxf,extent=[dist_s,dist_e,chirp_s,chirp_e]);self._image_plots_rc_filt.append(imf);cbf=self._fig.colorbar(imf,ax=self._axs[1,cf],label='Filt.Mag(dB)');self._colorbars_rc_filt.append(cbf);self._axs[1,cf].set_xlabel("Dist(m)");self._axs[1,cf].set_ylabel("ChirpIdx");self._axs[1,cf].set_title(f"Ant{i_ant} Filt Rng/Chirp")
            drdo=20*np.log10(drdo_ant+1e-9);vmro,vxro=np.percentile(drdo,[15,98]) if drdo.size>0 else(-80,-20);vxro=max(vxro, vmro+10);rdoim=self._axs[2,co].imshow(drdo,aspect='auto',cmap='jet',origin='lower',vmin=vmro,vmax=vxro,extent=[dist_s,dist_e,vel_s,vel_e]);self._range_doppler_plots_orig.append(rdoim);rdcbo=self._fig.colorbar(rdoim,ax=self._axs[2,co],label='Mag(dB)');self._range_doppler_colorbars_orig.append(rdcbo);self._axs[2,co].set_xlabel("Dist(m)");self._axs[2,co].set_ylabel("Vel(m/s)");self._axs[2,co].set_title(f"Ant{i_ant} Orig R-D")
            drdf=20*np.log10(drdf_ant+FILTERING_FLOOR_2D_MAG_LINEAR);vmrf,vxrf=np.percentile(drdf,[15,99]) if drdf.size>0 else(-90,-30);vxrf=max(vxrf, vmrf+10);rdfim=self._axs[2,cf].imshow(drdf,aspect='auto',cmap='jet',origin='lower',vmin=vmrf,vmax=vxrf,extent=[dist_s,dist_e,vel_s,vel_e]);self._range_doppler_plots_filt.append(rdfim);rdcbf=self._fig.colorbar(rdfim,ax=self._axs[2,cf],label='Filt.Mag(dB)');self._range_doppler_colorbars_filt.append(rdcbf)
            current_rd_filt_title = f"Ant {i_ant} Filt R-D"; title_color = 'black'
            if i_ant == 0:
                motion_status_str = "MOTION DETECTED" if self.app_state.motion_detected_status else "No Motion"
                triggering_ranges = [r for r, detected in self.app_state.motion_detected_status_by_range.items() if detected]
                range_info_str = f"\nTrg Rngs: {', '.join(triggering_ranges)}" if triggering_ranges else ""
                new_title = f"Ant 0 Filt R-D\n{motion_status_str}{range_info_str}"
                title_color = 'red' if self.app_state.motion_detected_status else 'black'
                self.motion_status_text_ant0 = self._axs[2,cf].set_title(new_title, color=title_color, fontsize=9)
            else: self._axs[2,cf].set_title(current_rd_filt_title, fontsize=10, color=title_color)
            self._axs[2,cf].set_xlabel("Dist(m)");self._axs[2,cf].set_ylabel("Vel(m/s)");
        self._fig.tight_layout(pad=1.0, h_pad=1.5, w_pad=0.5); plt.show(block=False)

    def _update_plots(self, data_1d_orig, data_2d_rc_orig, data_rd_orig, data_1d_filt, data_2d_rc_filt, data_rd_filt):
        min_yo,max_yo=self._get_1d_plot_ylimits(data_1d_orig);min_yf,max_yf=self._get_1d_plot_ylimits(data_1d_filt,True)
        for i_ant in range(self._num_ant):
            co,cf=i_ant*2,i_ant*2+1
            if i_ant<len(self._line_plots_orig) and i_ant<len(data_1d_orig) and data_1d_orig[i_ant].size > 0 : self._line_plots_orig[i_ant].set_ydata(data_1d_orig[i_ant]); self._axs[0,co].set_ylim(min_yo,max_yo)
            if i_ant<len(self._line_plots_filt) and i_ant<len(data_1d_filt) and data_1d_filt[i_ant].size > 0 : self._line_plots_filt[i_ant].set_ydata(data_1d_filt[i_ant]); self._axs[0,cf].set_ylim(min_yf,max_yf)
            if i_ant<len(self._image_plots_rc_orig) and i_ant<len(data_2d_rc_orig) and data_2d_rc_orig[i_ant].size > 0: d2o=20*np.log10(data_2d_rc_orig[i_ant]+1e-9);vmo,vxo=np.percentile(d2o,[5,95]);vxo=max(vxo,vmo+1); self._image_plots_rc_orig[i_ant].set_data(d2o);self._image_plots_rc_orig[i_ant].set_clim(vmin=vmo,vmax=vxo)
            if i_ant<len(self._image_plots_rc_filt) and i_ant<len(data_2d_rc_filt) and data_2d_rc_filt[i_ant].size > 0: d2f=20*np.log10(data_2d_rc_filt[i_ant]+FILTERING_FLOOR_2D_MAG_LINEAR);vmf,vxf=np.percentile(d2f,[5,98]);vxf=max(vxf,vmf+10); self._image_plots_rc_filt[i_ant].set_data(d2f);self._image_plots_rc_filt[i_ant].set_clim(vmin=vmf,vmax=vxf)
            if i_ant<len(self._range_doppler_plots_orig) and i_ant<len(data_rd_orig) and data_rd_orig[i_ant].size > 0: drdo=20*np.log10(data_rd_orig[i_ant]+1e-9);vmro,vxro=np.percentile(drdo,[15,98]);vxro=max(vxro,vmro+10); self._range_doppler_plots_orig[i_ant].set_data(drdo);self._range_doppler_plots_orig[i_ant].set_clim(vmin=vmro,vmax=vxro)
            if i_ant<len(self._range_doppler_plots_filt) and i_ant<len(data_rd_filt) and data_rd_filt[i_ant].size > 0: drdf=20*np.log10(data_rd_filt[i_ant]+FILTERING_FLOOR_2D_MAG_LINEAR);vmrf,vxrf=np.percentile(drdf,[15,99]);vxrf=max(vxrf,vmrf+10); self._range_doppler_plots_filt[i_ant].set_data(drdf);self._range_doppler_plots_filt[i_ant].set_clim(vmin=vmrf,vmax=vxrf)
            if i_ant == 0:
                ant0_rd_filt_ax = self._axs[2, cf]; base_title = f"Ant 0 Filt R-D"
                motion_status_str = "MOTION DETECTED" if self.app_state.motion_detected_status else "No Motion"
                triggering_ranges = [r for r, detected in self.app_state.motion_detected_status_by_range.items() if detected]
                range_info_str = f"\nTrg Rngs: {', '.join(triggering_ranges)}" if triggering_ranges else ""
                sust_info_str = ""
                if self.app_state.sustained_presence_enabled and any(g.get("is_magnified", False) for g in self.app_state.sustained_active_groups.values()): sust_info_str += " (S+)"
                if self.app_state.penalize_new_clusters_enabled and self.app_state.new_cluster_penalty_tracker: sust_info_str += " (P-)"
                new_title = f"{base_title}\n{motion_status_str}{range_info_str}{sust_info_str}"
                title_color = 'red' if self.app_state.motion_detected_status else 'black'
                if self.motion_status_text_ant0 and hasattr(self.motion_status_text_ant0, 'set_text'): self.motion_status_text_ant0.set_text(new_title); self.motion_status_text_ant0.set_color(title_color)
                elif len(ant0_rd_filt_ax.get_title()) > 0 : self.motion_status_text_ant0 = ant0_rd_filt_ax.set_title(new_title, color=title_color, fontsize=9)

    def draw(self,d1o,d2rco,drdo,d1f,d2rcf,drdf):
        if self._is_window_open:
            if not self._line_plots_orig:
                def safe_pad(data_list, num_ant, default_val_func):
                    return data_list + [default_val_func()] * (num_ant - len(data_list)) if len(data_list) < num_ant else data_list
                d1o_safe = safe_pad(d1o, self._num_ant, lambda: np.array([])); d2rco_safe = safe_pad(d2rco, self._num_ant, lambda: np.array([[]])); drdo_safe = safe_pad(drdo, self._num_ant, lambda: np.array([[]]))
                d1f_safe = safe_pad(d1f, self._num_ant, lambda: np.array([])); d2rcf_safe = safe_pad(d2rcf, self._num_ant, lambda: np.array([[]])); drdf_safe = safe_pad(drdf, self._num_ant, lambda: np.array([[]]))
                self._draw_first_time(d1o_safe, d2rco_safe, drdo_safe, d1f_safe, d2rcf_safe, drdf_safe)
            else: self._update_plots(d1o,d2rco,drdo,d1f,d2rcf,drdf)
            self._fig.canvas.draw_idle();self._fig.canvas.flush_events()
    def close(self,event=None):
        if self.is_open():self._is_window_open=False;plt.close(self._fig);plt.close('all')
    def is_open(self):return self._is_window_open

ser = None
rx_frame_channels_template=[np.zeros((NUM_CHIRPS_PER_FRAME if NUM_CHIRPS_PER_FRAME>0 else 1, NUM_SAMPLES_PER_CHIRP if NUM_SAMPLES_PER_CHIRP>0 else 1),dtype=np.float32) for _ in range(NUM_RX_ANTENNAS)]
def setup_serial(port, baud_rate):
    global ser
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.1)
        if not ser.is_open: ser.open()
        time.sleep(1); ser.reset_input_buffer()
        print(f"Serial port {port} opened successfully.")
        return True
    except serial.SerialException as e: print(f"Error serial port {port}: {e}"); ser = None; return False
    except Exception as e: print(f"Unexpected error serial setup {port}: {e}"); ser = None; return False

def parse_frame_data_from_serial(plot_is_open_flag_func):
    global ser;
    if not ser or not ser.is_open: return None,-1,True
    empty_l_ctr,max_empty_l=0,10
    while plot_is_open_flag_func():
        try:
            line_b=ser.readline();
            if not line_b:
                empty_l_ctr+=1
                if empty_l_ctr>max_empty_l: print_once("Max empty lines on serial. Check device."); return None,-1,False
                time.sleep(0.01); continue
            empty_l_ctr=0; line=line_b.decode('utf-8',errors='replace').strip()
            match=re.match(r"Frame (\d+): \[(.*)\]",line)
            if match:
                f_num_s,dat_s=match.groups()
                try: flat_d1=np.array([float(x) for x in dat_s.split(', ')],dtype=np.float32)
                except ValueError: continue
                if len(flat_d1)==EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    curr_rx_f=[np.zeros_like(tpl) for tpl in rx_frame_channels_template]; idx,resh_ok=0,True
                    for c_i in range(NUM_CHIRPS_PER_FRAME if NUM_CHIRPS_PER_FRAME > 0 else 1):
                        for s_i in range(NUM_SAMPLES_PER_CHIRP if NUM_SAMPLES_PER_CHIRP > 0 else 1):
                            for r_ant_i in range(NUM_RX_ANTENNAS):
                                if idx<len(flat_d1): curr_rx_f[r_ant_i][c_i,s_i]=flat_d1[idx]; idx+=1
                                else: resh_ok=False; break
                            if not resh_ok: break
                        if not resh_ok: break
                    if resh_ok: return curr_rx_f,int(f_num_s),False
            continue
        except serial.SerialException as se: print(f"SerialException parse: {se}"); return None,-1,True
        except UnicodeDecodeError as ue: print_once(f"UnicodeDecodeError parse: {ue}"); continue
        except Exception as e: traceback.print_exc(); return None,-1,True
    return None,-1, (not plot_is_open_flag_func())

_printed_warnings = set()
def print_once(message):
    if message not in _printed_warnings: print(message); _printed_warnings.add(message)

def find_pixel_groups(hot_pixel_mask, min_pixels_for_group):
    if hot_pixel_mask.size == 0 or np.sum(hot_pixel_mask) == 0: return []
    labeled_array, num_features = label(hot_pixel_mask)
    groups = []
    for i in range(1, num_features + 1):
        pixels_in_group = np.where(labeled_array == i)
        num_pixels = len(pixels_in_group[0])
        if num_pixels >= min_pixels_for_group:
            centroid_r = np.mean(pixels_in_group[0]); centroid_d = np.mean(pixels_in_group[1])
            groups.append({"centroid": (centroid_r, centroid_d), "num_pixels": num_pixels, "pixels_indices": list(zip(pixels_in_group[0], pixels_in_group[1]))})
    return groups

def update_sustained_groups_and_penalties(app_state: AppState, hot_pixel_map_for_grouping: np.ndarray, current_frame_id: int, dist_points_axis: np.ndarray):
    effective_pixel_counts_per_range_bin = np.zeros(hot_pixel_map_for_grouping.shape[1]) 
    current_time = time.time()
    current_frame_pixel_groups_props = find_pixel_groups(hot_pixel_map_for_grouping, app_state.sustained_min_hot_pixels_for_group)
    matched_current_group_indices = [False] * len(current_frame_pixel_groups_props)
    max_centroid_shift_for_match = app_state.sustained_pixel_proximity_threshold * 1.5 
    new_sustained_active_groups = {}
    for group_id, sustained_info in list(app_state.sustained_active_groups.items()):
        best_match_idx = -1; min_dist = float('inf')
        for i, current_group_prop in enumerate(current_frame_pixel_groups_props):
            if matched_current_group_indices[i]: continue
            dist = np.sqrt( (sustained_info["centroid"][0] - current_group_prop["centroid"][0])**2 + (sustained_info["centroid"][1] - current_group_prop["centroid"][1])**2 )
            if dist <= max_centroid_shift_for_match and dist < min_dist: min_dist = dist; best_match_idx = i
        if best_match_idx != -1:
            matched_current_group_indices[best_match_idx] = True
            matched_group_data = current_frame_pixel_groups_props[best_match_idx]
            sustained_info.update({"frames_seen": sustained_info["frames_seen"] + 1, "last_seen_time": current_time, "gap_frames": 0, "centroid": matched_group_data["centroid"]})
            pixel_contribution = matched_group_data["num_pixels"]
            if app_state.sustained_presence_enabled and (sustained_info["last_seen_time"] - sustained_info["first_seen_time"]) >= app_state.sustained_duration_threshold_s:
                sustained_info["is_magnified"] = True
                pixel_contribution *= app_state.sustained_magnification_factor
            for _, d_idx in matched_group_data["pixels_indices"]: 
                if 0 <= d_idx < len(effective_pixel_counts_per_range_bin):
                    effective_pixel_counts_per_range_bin[d_idx] += (pixel_contribution / matched_group_data["num_pixels"]) 
            new_sustained_active_groups[group_id] = sustained_info
        else:
            sustained_info["gap_frames"] += 1
            if sustained_info["gap_frames"] <= app_state.sustained_max_gap_frames: new_sustained_active_groups[group_id] = sustained_info
            else: print_once(f"Sustained group {group_id} lost.")
    app_state.sustained_active_groups = new_sustained_active_groups
    new_penalty_tracker = {}
    for group_id, penalty_info in list(app_state.new_cluster_penalty_tracker.items()):
        best_match_idx = -1; min_dist = float('inf')
        for i, current_group_prop in enumerate(current_frame_pixel_groups_props):
            if matched_current_group_indices[i]: continue
            dist = np.sqrt( (penalty_info["centroid"][0] - current_group_prop["centroid"][0])**2 + (penalty_info["centroid"][1] - current_group_prop["centroid"][1])**2 )
            if dist <= max_centroid_shift_for_match and dist < min_dist: min_dist = dist; best_match_idx = i
        if best_match_idx != -1:
            matched_current_group_indices[best_match_idx] = True
            matched_group_data = current_frame_pixel_groups_props[best_match_idx]
            penalty_info.update({"frames_penalized": penalty_info["frames_penalized"] + 1, "centroid": matched_group_data["centroid"]})
            pixel_contribution = matched_group_data["num_pixels"]
            if penalty_info["frames_penalized"] <= app_state.penalize_frames_duration:
                pixel_contribution *= (1.0 - app_state.penalize_reduction_factor)
                new_penalty_tracker[group_id] = penalty_info 
            else: print_once(f"Penalty expired for group {group_id}.")
            for _, d_idx in matched_group_data["pixels_indices"]:
                 if 0 <= d_idx < len(effective_pixel_counts_per_range_bin):
                    effective_pixel_counts_per_range_bin[d_idx] += (pixel_contribution / matched_group_data["num_pixels"])
    app_state.new_cluster_penalty_tracker = new_penalty_tracker
    for i, current_group_prop in enumerate(current_frame_pixel_groups_props):
        if not matched_current_group_indices[i]: 
            pixel_contribution = current_group_prop["num_pixels"]
            if app_state.penalize_new_clusters_enabled:
                new_penalty_id = app_state.next_penalty_group_id; app_state.next_penalty_group_id += 1
                app_state.new_cluster_penalty_tracker[new_penalty_id] = {"centroid": current_group_prop["centroid"], "frames_penalized": 1}
                pixel_contribution *= (1.0 - app_state.penalize_reduction_factor)
            if app_state.sustained_presence_enabled :
                new_sust_id = app_state.next_sustained_group_id; app_state.next_sustained_group_id += 1
                app_state.sustained_active_groups[new_sust_id] = {
                    "centroid": current_group_prop["centroid"], "frames_seen": 1, "last_seen_time": current_time, 
                    "gap_frames": 0, "first_seen_time": current_time, "is_magnified": False
                }
            for _, d_idx in current_group_prop["pixels_indices"]:
                if 0 <= d_idx < len(effective_pixel_counts_per_range_bin):
                    effective_pixel_counts_per_range_bin[d_idx] += (pixel_contribution / current_group_prop["num_pixels"])
    app_state.debug_effective_pixels_per_bin = effective_pixel_counts_per_range_bin 
    return effective_pixel_counts_per_range_bin

if __name__ == '__main__':
    app_state = AppState()
    load_configuration(app_state, CONFIG_FILE_PATH)

    threshold_gui_root = None
    if not setup_serial(SERIAL_PORT, BAUD_RATE):
        parent_win = app_state.gui_root if app_state.gui_is_active and app_state.gui_root else None
        if not parent_win: temp_root = tk.Tk(); temp_root.withdraw(); parent_win = temp_root
        messagebox.showerror("Serial Error", f"Could not open serial port {SERIAL_PORT}.", parent=parent_win)
        if 'temp_root' in locals(): temp_root.destroy()
        exit()
    
    try:
        threshold_gui_root = setup_threshold_gui(app_state)
        update_gui_from_appstate(app_state) 
    except tk.TclError as e:
        print(f"Tkinter GUI Error: {e}. Running without threshold GUI.")
        app_state.gui_is_active = False; threshold_gui_root = None

    radar_config = RadarConfig(NUM_CHIRPS_PER_FRAME,NUM_SAMPLES_PER_CHIRP,START_FREQUENCY_HZ,END_FREQUENCY_HZ,PROCESSING_FRAMERATE_HZ,SAMPLE_RATE_HZ,CHIRP_REPETITION_TIME_S)
    distance_algo = DistanceFFT_Algo(radar_config)
    plotter = Draw(radar_config,0,NUM_RX_ANTENNAS,distance_algo._range_bin_length,distance_algo._velocity_axis_mps,app_state)
    frame_delay_s = 1.0/PROCESSING_FRAMERATE_HZ if PROCESSING_FRAMERATE_HZ>0 else 0.01; frame_count=0; initial_plot_drawn = False

    try:
        while True:
            current_loop_time = time.time()
            if not plotter.is_open(): print("Plot window closed. Exiting."); break
            if app_state.gui_is_active and threshold_gui_root:
                try: threshold_gui_root.update_idletasks(); threshold_gui_root.update()
                except tk.TclError:
                    if app_state.gui_is_active: app_state.gui_is_active=False; threshold_gui_root=None
                    print_once("Threshold GUI closed or error.")
            
            if app_state.auto_calibration_enabled and plotter.is_open() and (current_loop_time - app_state.last_calibration_time)>app_state.auto_calibration_interval_s:
                print(f"Auto-calibrating..."); app_state.calibration_requested=True

            start_t_f = time.time()
            ant_frames,parsed_f_num,should_exit_parser = parse_frame_data_from_serial(plotter.is_open)

            if not plotter.is_open(): break
            if should_exit_parser : print("Parser indicated exit. Shutting down."); break

            if ant_frames is not None:
                initial_plot_drawn = True; frame_count+=1
                all_ant_1d_orig,all_ant_2d_rc_orig,all_ant_rd_orig = [[] for _ in range(3)]
                for i_ant_proc in range(NUM_RX_ANTENNAS):
                    raw_data_ant = ant_frames[i_ant_proc] if i_ant_proc < len(ant_frames) else np.zeros((NUM_CHIRPS_PER_FRAME or 1, NUM_SAMPLES_PER_CHIRP or 1))
                    dist_p,prof1d,fft_abs2d,rd_map_abs = distance_algo.compute_distance_and_profile(raw_data_ant)
                    all_ant_1d_orig.append(prof1d); all_ant_2d_rc_orig.append(fft_abs2d); all_ant_rd_orig.append(rd_map_abs)

                if app_state.calibration_requested:
                    app_state.baseline_1d_profiles = [p.copy() if p is not None else None for p in all_ant_1d_orig[:NUM_RX_ANTENNAS]]
                    app_state.baseline_2d_fft_abs = [p.copy() if p is not None else None for p in all_ant_2d_rc_orig[:NUM_RX_ANTENNAS]]
                    app_state.baseline_rd_maps_abs = [p.copy() if p is not None else None for p in all_ant_rd_orig[:NUM_RX_ANTENNAS]]
                    app_state.filtering_enabled=True; app_state.calibration_requested=False
                    app_state.last_calibration_time = time.time(); print("Baseline calibrated.")

                hot_pixel_map_for_grouping = np.zeros_like(all_ant_rd_orig[0])
                if app_state.filtering_enabled and NUM_RX_ANTENNAS > 0 and all_ant_rd_orig[0] is not None and \
                   app_state.baseline_rd_maps_abs[0] is not None and \
                   all_ant_rd_orig[0].shape == app_state.baseline_rd_maps_abs[0].shape:
                    subtracted_rd_ant0 = all_ant_rd_orig[0] - app_state.baseline_rd_maps_abs[0]
                    temp_filt_rd_after_2d_filter = np.full_like(subtracted_rd_ant0, FILTERING_FLOOR_2D_MAG_LINEAR)
                    ant0_2d_filt_thresh_map = app_state.per_antenna_range_thresholds_2d[0]

                    for bin_j in range(subtracted_rd_ant0.shape[1]):
                        dist_m = plotter._dist_points[bin_j] if hasattr(plotter, '_dist_points') and bin_j < len(plotter._dist_points) else -1
                        current_2d_filter_thresh = FILTERING_FLOOR_2D_MAG_LINEAR
                        if dist_m != -1:
                            if dist_m >= app_state.range_definitions["8.0m+"][0]:
                                current_2d_filter_thresh = ant0_2d_filt_thresh_map.get("8.0m+", FILTERING_FLOOR_2D_MAG_LINEAR)
                            else:
                                for r_key,(min_r,max_r) in app_state.range_definitions.items():
                                    if min_r <= dist_m < max_r:
                                        current_2d_filter_thresh = ant0_2d_filt_thresh_map.get(r_key, FILTERING_FLOOR_2D_MAG_LINEAR)
                                        break
                        temp_filt_rd_after_2d_filter[:,bin_j] = np.where(subtracted_rd_ant0[:,bin_j] >= current_2d_filter_thresh, subtracted_rd_ant0[:,bin_j], FILTERING_FLOOR_2D_MAG_LINEAR)
                    
                    for bin_j in range(temp_filt_rd_after_2d_filter.shape[1]):
                        dist_m = plotter._dist_points[bin_j] if hasattr(plotter, '_dist_points') and bin_j < len(plotter._dist_points) else -1
                        current_motion_mag_thresh = app_state.motion_pixel_magnitude_thresholds.get("0.0-0.5m", 0.01) 
                        if dist_m != -1:
                            if dist_m >= app_state.range_definitions["8.0m+"][0]:
                                current_motion_mag_thresh = app_state.motion_pixel_magnitude_thresholds.get("8.0m+", current_motion_mag_thresh)
                            else:
                                for r_key,(min_r,max_r) in app_state.range_definitions.items():
                                    if min_r <= dist_m < max_r:
                                        current_motion_mag_thresh = app_state.motion_pixel_magnitude_thresholds.get(r_key, current_motion_mag_thresh)
                                        break
                        hot_pixel_map_for_grouping[:,bin_j] = temp_filt_rd_after_2d_filter[:,bin_j] >= current_motion_mag_thresh
                
                effective_pixels_per_range_bin = update_sustained_groups_and_penalties(app_state, hot_pixel_map_for_grouping, frame_count, plotter._dist_points if hasattr(plotter, '_dist_points') else np.array([]))

                any_range_detected_this_frame = False
                # <<< MODIFIED: Determine color for HA based on triggering range >>>
                triggering_color = None 
                # Iterate in the order of range_definitions keys (Python 3.7+ preserves insertion order)
                # This typically means checking from closest to furthest range.
                for r_key, (min_dist_m, max_dist_m) in app_state.range_definitions.items():
                    range_zone_bins_indices = []
                    if hasattr(plotter, '_dist_points') and plotter._dist_points.ndim == 1:
                        if r_key == "8.0m+":
                             range_zone_bins_indices = [idx for idx, dist_val in enumerate(plotter._dist_points) if dist_val >= min_dist_m]
                        else:
                             range_zone_bins_indices = [idx for idx, dist_val in enumerate(plotter._dist_points) if min_dist_m <= dist_val < max_dist_m]
                    else:
                        print_once("Warning: plotter._dist_points not available or not 1D for range zone mapping.")
                    
                    pixels_in_this_range_zone = np.sum(effective_pixels_per_range_bin[range_zone_bins_indices]) if range_zone_bins_indices and len(range_zone_bins_indices) <= len(effective_pixels_per_range_bin) else 0
                    
                    current_range_count_thresh = app_state.motion_pixel_count_thresholds.get(r_key, 10)
                    current_range_frames_thresh = app_state.motion_consecutive_frames_thresholds.get(r_key, 2)

                    if pixels_in_this_range_zone >= current_range_count_thresh:
                        app_state.motion_consecutive_frames_current_count_by_range[r_key] += 1
                        app_state.motion_current_gap_count_by_range[r_key] = 0
                    else:
                        if app_state.motion_consecutive_frames_current_count_by_range[r_key] > 0:
                            app_state.motion_current_gap_count_by_range[r_key] += 1
                            if app_state.motion_current_gap_count_by_range[r_key] > app_state.motion_allowed_gaps:
                                app_state.motion_consecutive_frames_current_count_by_range[r_key] = 0
                                app_state.motion_current_gap_count_by_range[r_key] = 0
                    
                    if app_state.motion_consecutive_frames_current_count_by_range[r_key] >= current_range_frames_thresh:
                        if not app_state.motion_detected_status_by_range[r_key]:
                            print_once(f"--- MOTION DETECTED in range {r_key} ({time.strftime('%H:%M:%S')}) ---")
                        app_state.motion_detected_status_by_range[r_key] = True
                        app_state.motion_last_detected_time_by_range[r_key] = time.time()
                        any_range_detected_this_frame = True
                        if triggering_color is None: # Set color based on the first (closest) triggering range
                            triggering_color = app_state.range_to_color_rgb.get(r_key)
                    elif app_state.motion_detected_status_by_range[r_key]: 
                         if app_state.motion_consecutive_frames_current_count_by_range[r_key] < current_range_frames_thresh:
                            print_once(f"--- No Motion in range {r_key} (criteria no longer met) ---")
                            app_state.motion_detected_status_by_range[r_key] = False
                            app_state.motion_consecutive_frames_current_count_by_range[r_key] = 0 
                            app_state.motion_current_gap_count_by_range[r_key] = 0
                
                previous_overall_motion_status = app_state.motion_detected_status
                current_overall_motion_status_based_on_ranges = any(app_state.motion_detected_status_by_range.values())
                
                if current_overall_motion_status_based_on_ranges:
                    if not app_state.motion_detected_status: # Transition to ON
                        print(f"--- OVERALL MOTION DETECTED ({time.strftime('%H:%M:%S')}) ---")
                        # <<< MODIFIED: Pass color to HA light control >>>
                        if app_state.home_assistant_enabled:
                             control_home_assistant_light(app_state, turn_on=True, rgb_color=triggering_color)
                    elif app_state.motion_detected_status and triggering_color != app_state.current_light_color:
                        # Motion ongoing, but detected range (and thus color) has changed
                        print(f"--- OVERALL MOTION CONTINUES, color updated for new range ({time.strftime('%H:%M:%S')}) ---")
                        if app_state.home_assistant_enabled:
                            control_home_assistant_light(app_state, turn_on=True, rgb_color=triggering_color)
                            
                    app_state.motion_detected_status = True
                    app_state.motion_last_overall_detected_time = time.time()
                else: 
                    if app_state.motion_detected_status: 
                        if (time.time() - app_state.motion_last_overall_detected_time) > app_state.motion_hold_time_s:
                            print(f"--- OVERALL No Motion (Hold Time Expired at {time.strftime('%H:%M:%S')}) ---")
                            app_state.motion_detected_status = False
                            # <<< MODIFIED: Pass no color when turning off >>>
                            if app_state.home_assistant_enabled and app_state.light_is_on: 
                                control_home_assistant_light(app_state, turn_on=False) # No color needed for turn_off
                            if previous_overall_motion_status and not app_state.motion_detected_status:
                                print("Overall motion stopped. Requesting recalibration.")
                                app_state.calibration_requested = True
                
                all_ant_1d_filt, all_ant_2d_rc_filt, all_ant_rd_filt = [[] for _ in range(3)]
                if app_state.filtering_enabled:
                    for i_ant_plot in range(NUM_RX_ANTENNAS):
                        baseline_1d_ok = i_ant_plot < len(app_state.baseline_1d_profiles) and app_state.baseline_1d_profiles[i_ant_plot] is not None and \
                                         i_ant_plot < len(all_ant_1d_orig) and all_ant_1d_orig[i_ant_plot].shape == app_state.baseline_1d_profiles[i_ant_plot].shape
                        baseline_2d_rc_ok = i_ant_plot < len(app_state.baseline_2d_fft_abs) and app_state.baseline_2d_fft_abs[i_ant_plot] is not None and \
                                            i_ant_plot < len(all_ant_2d_rc_orig) and all_ant_2d_rc_orig[i_ant_plot].shape == app_state.baseline_2d_fft_abs[i_ant_plot].shape
                        baseline_rd_ok = i_ant_plot < len(app_state.baseline_rd_maps_abs) and app_state.baseline_rd_maps_abs[i_ant_plot] is not None and \
                                         i_ant_plot < len(all_ant_rd_orig) and all_ant_rd_orig[i_ant_plot].shape == app_state.baseline_rd_maps_abs[i_ant_plot].shape
                        
                        if baseline_1d_ok :
                            sub_1d=all_ant_1d_orig[i_ant_plot]-app_state.baseline_1d_profiles[i_ant_plot];filt1d_prof=np.full_like(sub_1d,FILTERING_FLOOR_1D_PROFILE_LINEAR);ant_thresh_1d_ranges=app_state.per_antenna_range_thresholds_1d[i_ant_plot]
                            if hasattr(plotter,'_dist_points') and len(plotter._dist_points)==len(sub_1d):
                                for bin_i,dist_m_plot in enumerate(plotter._dist_points):
                                    app_thr_1d_plot=FILTERING_FLOOR_1D_PROFILE_LINEAR 
                                    if dist_m_plot >= app_state.range_definitions["8.0m+"][0]: 
                                        app_thr_1d_plot=ant_thresh_1d_ranges.get("8.0m+", FILTERING_FLOOR_1D_PROFILE_LINEAR)
                                    else:
                                        for r_key_plot,(min_r_plot,max_r_plot) in app_state.range_definitions.items():
                                            if min_r_plot<=dist_m_plot<max_r_plot:
                                                app_thr_1d_plot=ant_thresh_1d_ranges.get(r_key_plot, FILTERING_FLOOR_1D_PROFILE_LINEAR); break
                                    if sub_1d[bin_i]>=app_thr_1d_plot:filt1d_prof[bin_i]=sub_1d[bin_i]
                            else:print_once(f"W:1D dist_points ant{i_ant_plot} mismatch or unavailable");fallback_thr_1d_plot=next(iter(ant_thresh_1d_ranges.values()),0.01);temp_f1d_plot=sub_1d.copy();temp_f1d_plot[temp_f1d_plot<fallback_thr_1d_plot]=FILTERING_FLOOR_1D_PROFILE_LINEAR;filt1d_prof=temp_f1d_plot
                            all_ant_1d_filt.append(filt1d_prof)
                        else: all_ant_1d_filt.append(np.zeros(NUM_SAMPLES_PER_CHIRP or 1))
                        
                        ant_thresh_2d_ranges_plot=app_state.per_antenna_range_thresholds_2d[i_ant_plot]
                        if baseline_2d_rc_ok:
                            sub_2d_rc=all_ant_2d_rc_orig[i_ant_plot]-app_state.baseline_2d_fft_abs[i_ant_plot];filt2d_rc_plot=np.full_like(sub_2d_rc,FILTERING_FLOOR_2D_MAG_LINEAR)
                            if hasattr(plotter,'_dist_points') and plotter._dist_points.shape[0]==sub_2d_rc.shape[1]:
                                for bin_j_plot in range(sub_2d_rc.shape[1]):
                                    dist_m_plot=plotter._dist_points[bin_j_plot];app_thr_2d_plot=FILTERING_FLOOR_2D_MAG_LINEAR
                                    if dist_m_plot >= app_state.range_definitions["8.0m+"][0]:
                                        app_thr_2d_plot=ant_thresh_2d_ranges_plot.get("8.0m+",FILTERING_FLOOR_2D_MAG_LINEAR)
                                    else:
                                        for r_key_plot,(min_r_plot,max_r_plot) in app_state.range_definitions.items():
                                            if min_r_plot<=dist_m_plot<max_r_plot:
                                                app_thr_2d_plot=ant_thresh_2d_ranges_plot.get(r_key_plot,FILTERING_FLOOR_2D_MAG_LINEAR);break
                                    col_data_plot=sub_2d_rc[:,bin_j_plot];filt2d_rc_plot[:,bin_j_plot]=np.where(col_data_plot>=app_thr_2d_plot,col_data_plot,FILTERING_FLOOR_2D_MAG_LINEAR)
                            else:print_once(f"W:2D RC dist_points ant{i_ant_plot} mismatch");fallback_thr_2d_plot=next(iter(ant_thresh_2d_ranges_plot.values()),0.001);filt2d_rc_plot=sub_2d_rc.copy();filt2d_rc_plot[filt2d_rc_plot<fallback_thr_2d_plot]=FILTERING_FLOOR_2D_MAG_LINEAR
                            all_ant_2d_rc_filt.append(filt2d_rc_plot)
                        else: all_ant_2d_rc_filt.append(np.full((NUM_CHIRPS_PER_FRAME or 1,NUM_SAMPLES_PER_CHIRP or 1),FILTERING_FLOOR_2D_MAG_LINEAR))
                        
                        if baseline_rd_ok:
                            sub_rd=all_ant_rd_orig[i_ant_plot]-app_state.baseline_rd_maps_abs[i_ant_plot];filt_rd_map_plot=np.full_like(sub_rd,FILTERING_FLOOR_2D_MAG_LINEAR)
                            if hasattr(plotter,'_dist_points') and plotter._dist_points.shape[0]==sub_rd.shape[1]:
                                 for bin_j_plot in range(sub_rd.shape[1]):
                                    dist_m_plot=plotter._dist_points[bin_j_plot];app_thr_2d_plot=FILTERING_FLOOR_2D_MAG_LINEAR
                                    if dist_m_plot >= app_state.range_definitions["8.0m+"][0]:
                                        app_thr_2d_plot=ant_thresh_2d_ranges_plot.get("8.0m+",FILTERING_FLOOR_2D_MAG_LINEAR)
                                    else:
                                        for r_key_plot,(min_r_plot,max_r_plot) in app_state.range_definitions.items():
                                            if min_r_plot<=dist_m_plot<max_r_plot:
                                                app_thr_2d_plot=ant_thresh_2d_ranges_plot.get(r_key_plot,FILTERING_FLOOR_2D_MAG_LINEAR);break
                                    col_data_plot=sub_rd[:,bin_j_plot];filt_rd_map_plot[:,bin_j_plot]=np.where(col_data_plot>=app_thr_2d_plot,col_data_plot,FILTERING_FLOOR_2D_MAG_LINEAR)
                            else:print_once(f"W:R-D dist_points ant{i_ant_plot} mismatch");fallback_thr_2d_plot=next(iter(ant_thresh_2d_ranges_plot.values()),0.001);filt_rd_map_plot=sub_rd.copy();filt_rd_map_plot[filt_rd_map_plot<fallback_thr_2d_plot]=FILTERING_FLOOR_2D_MAG_LINEAR
                            all_ant_rd_filt.append(filt_rd_map_plot)
                        else: all_ant_rd_filt.append(np.full((NUM_CHIRPS_PER_FRAME or 1,NUM_SAMPLES_PER_CHIRP or 1),FILTERING_FLOOR_2D_MAG_LINEAR))
                else: 
                    for _ in range(NUM_RX_ANTENNAS):
                        all_ant_1d_filt.append(np.zeros(NUM_SAMPLES_PER_CHIRP or 1)); all_ant_2d_rc_filt.append(np.full((NUM_CHIRPS_PER_FRAME or 1, NUM_SAMPLES_PER_CHIRP or 1),FILTERING_FLOOR_2D_MAG_LINEAR)); all_ant_rd_filt.append(np.full((NUM_CHIRPS_PER_FRAME or 1, NUM_SAMPLES_PER_CHIRP or 1),FILTERING_FLOOR_2D_MAG_LINEAR))

                if plotter.is_open(): plotter.draw(all_ant_1d_orig,all_ant_2d_rc_orig,all_ant_rd_orig,all_ant_1d_filt,all_ant_2d_rc_filt,all_ant_rd_filt)
            
            elif initial_plot_drawn and plotter.is_open(): plotter._fig.canvas.flush_events()
            elif not initial_plot_drawn and plotter.is_open():
                print_once("No data yet, drawing placeholder plots."); placeholder_s=NUM_SAMPLES_PER_CHIRP or 1; placeholder_c=NUM_CHIRPS_PER_FRAME or 1
                placeholder_1d=[np.zeros(placeholder_s) for _ in range(NUM_RX_ANTENNAS)]; placeholder_2d=[np.zeros((placeholder_c, placeholder_s)) for _ in range(NUM_RX_ANTENNAS)]
                plotter.draw(placeholder_1d,placeholder_2d,placeholder_2d,placeholder_1d,placeholder_2d,placeholder_2d); initial_plot_drawn=True
            
            elap_t_f = time.time()-start_t_f; sleep_dur = frame_delay_s-elap_t_f
            if sleep_dur > 0: time.sleep(sleep_dur)
            else: time.sleep(0.001) 

    except KeyboardInterrupt: print("\nData collection stopped by user (Ctrl+C).")
    except Exception as e: print("\n--- UNEXPECTED ERROR ---"); traceback.print_exc(); print("--------------------------")
    finally:
        if plotter.is_open(): plotter.close()
        if threshold_gui_root and app_state.gui_is_active:
            try: 
                if threshold_gui_root.winfo_exists(): threshold_gui_root.destroy()
            except tk.TclError: pass 
            app_state.gui_is_active = False
        if ser and ser.is_open: ser.close(); print("Serial port closed.")
        if app_state.home_assistant_enabled and app_state.light_is_on:
            print("Turning off light on exit..."); 
            control_home_assistant_light(app_state, turn_on=False) # Ensure light turns off
        print("Program finished.")