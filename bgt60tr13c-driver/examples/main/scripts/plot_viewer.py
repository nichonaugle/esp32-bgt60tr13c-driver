import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests 
import time

# =============================================================================
# --- Configuration ---
# =============================================================================
# !!! IMPORTANT: Change this to the correct serial port for your ESP32 !!!
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART'  
BAUD_RATE = 921600

# --- Radar & Plot Parameters ---
# These parameters MUST match the values in your radar_config.h
C_MPS = 299792458.0
FS_HZ = 2352941.0
PY_TC_S = 0.00005738
PY_F_BANDWIDTH_HZ = 957000000.0
N_RANGE_BINS = 512 

# --- Range Calculation ---
MAX_RANGE_M = (C_MPS * FS_HZ * PY_TC_S) / (4.0 * PY_F_BANDWIDTH_HZ)
print(f"INFO: Calculated plot range (MAX_RANGE_M) = {MAX_RANGE_M:.2f} m")

# =============================================================================
# --- Home Assistant Configuration ---
# =============================================================================
HA_ENABLED = True # Set to False to disable Home Assistant integration
HA_URL = "http://homeassistant.local:8123"
HA_TOKEN = "lol i'm not leaking my token again"
HA_ENTITY_ID = "light.smart_multicolor_bulb"

# The time delay for turning off is now handled by the ESP32 firmware.
# This script simply reacts to the occupancy state it receives.

# =============================================================================
# --- Global Variables ---
# =============================================================================
ser = None
range_axis = np.linspace(0, MAX_RANGE_M, N_RANGE_BINS)
latest_range_profile = np.full(N_RANGE_BINS, np.nan)
latest_cfar_threshold = np.full(N_RANGE_BINS, np.nan)
latest_detected_ranges = np.array([])
latest_detected_magnitudes = np.array([])
latest_occupancy_state = False # Tracks the occupancy state from the sensor

# =============================================================================
# --- Plotting Setup ---
# =============================================================================
plt.style.use('default')
fig, ax = plt.subplots(figsize=(12, 7))
line_profile, = ax.plot(range_axis, np.zeros(N_RANGE_BINS), 'g-', lw=1.0, label='Range Profile (dB)')
line_thresh, = ax.plot(range_axis, np.zeros(N_RANGE_BINS), 'y--', lw=1.5, label='CFAR Threshold (dB)')
line_targets, = ax.plot([], [], 'ro', markersize=8, label='Confirmed Detections')
ax.set_title('ESP32 Live Radar Data')
ax.set_xlabel('Range (m)')
ax.set_ylabel('Magnitude (dB)')
ax.set_xlim(0, MAX_RANGE_M)
ax.grid(True)
ax.legend(loc='upper right')

# =============================================================================
# --- Home Assistant Control Functions ---
# =============================================================================
def get_home_assistant_entity_state():
    """Queries the Home Assistant API to get the current state of the entity."""
    if not HA_ENABLED:
        return None

    api_url = f"{HA_URL}/api/states/{HA_ENTITY_ID}"
    headers = {
        "Authorization": f"Bearer {HA_TOKEN}",
        "Content-Type": "application/json",
    }
    
    try:
        response = requests.get(api_url, headers=headers, timeout=2)
        response.raise_for_status()
        return response.json().get('state')
    except requests.exceptions.RequestException as e:
        print(f"\nERROR: Could not get state from Home Assistant. Details: {e}")
        return None

def set_home_assistant_entity_state(state):
    """Calls the Home Assistant API to turn an entity 'on' or 'off'."""
    if not HA_ENABLED:
        return

    entity_domain = HA_ENTITY_ID.split('.')[0]
    service = f"turn_{state}"
    
    api_url = f"{HA_URL}/api/services/{entity_domain}/{service}"
    headers = {
        "Authorization": f"Bearer {HA_TOKEN}",
        "Content-Type": "application/json",
    }
    data = {"entity_id": HA_ENTITY_ID}
    
    try:
        response = requests.post(api_url, headers=headers, json=data, timeout=2)
        response.raise_for_status()
        print(f"\nINFO: Home Assistant - Successfully sent command to turn {HA_ENTITY_ID} {state.upper()}")
    except requests.exceptions.RequestException as e:
        print(f"\nERROR: Could not set state on Home Assistant. Details: {e}")

# =============================================================================
# --- Core Logic ---
# =============================================================================
def connect_serial():
    """Tries to connect to the configured serial port."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT}")
        ser.reset_input_buffer()
        return True
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. Please check the port name and permissions.")
        print(f"Details: {e}")
        return False

def parse_data_line(line_str):
    """Parses a line of comma-separated values into a numpy array."""
    try:
        data_part = line_str.split(':')[1].strip()
        if not data_part: return np.array([])
        parts = data_part.split(',')
        return np.array([float(p) if p != 'nan' else np.nan for p in parts])
    except (ValueError, IndexError) as e:
        print(f"Warning: Could not parse line: {line_str.strip()}. Error: {e}")
        return None

def read_serial_data():
    """Reads and processes one complete data block from the ESP32."""
    global latest_range_profile, latest_cfar_threshold, latest_detected_ranges
    global latest_detected_magnitudes, latest_occupancy_state
    
    in_plot_block = False
    
    while ser and ser.is_open:
        try:
            line_bytes = ser.readline()
            if not line_bytes: return 

            line = line_bytes.decode('utf-8', errors='ignore').strip()

            if "---PLOT_START---" in line:
                in_plot_block = True
                temp_profile, temp_cfar, temp_detections, temp_occupancy = None, None, None, None
                continue

            if in_plot_block:
                if line.startswith("RANGE_PROFILE_DB:"):
                    temp_profile = parse_data_line(line)
                elif line.startswith("CFAR_THRESHOLD_DB:"):
                    temp_cfar = parse_data_line(line)
                elif line.startswith("DETECTIONS_RANGES_M:"):
                    temp_detections = parse_data_line(line)
                elif line.startswith("OCCUPANCY_STATE:"):
                    try:
                        temp_occupancy = (line.strip().split(':')[1] == '1')
                    except IndexError:
                        temp_occupancy = None
                elif "---PLOT_END---" in line:
                    in_plot_block = False
                    
                    # --- Update global data buffers ---
                    if temp_profile is not None: latest_range_profile = temp_profile
                    if temp_cfar is not None: latest_cfar_threshold = temp_cfar
                    if temp_occupancy is not None: latest_occupancy_state = temp_occupancy
                    
                    if temp_detections is not None:
                        latest_detected_ranges = temp_detections
                        if temp_detections.size > 0:
                            # Find magnitudes for plotting the red dots
                            indices = np.searchsorted(range_axis, temp_detections)
                            indices = np.clip(indices, 0, N_RANGE_BINS - 1)
                            latest_detected_magnitudes = latest_range_profile[indices]
                        else:
                            latest_detected_magnitudes = np.array([])
                    else:
                         latest_detected_ranges = np.array([])
                         latest_detected_magnitudes = np.array([])

                    # --- Home Assistant Logic ---
                    if HA_ENABLED:
                        actual_state = get_home_assistant_entity_state()
                        
                        # If occupancy is detected and the light is off, turn it on.
                        if latest_occupancy_state and actual_state == 'off':
                            print(f"\nINFO: Occupancy detected. Turning ON {HA_ENTITY_ID}")
                            set_home_assistant_entity_state('on')
                        # If occupancy is NOT detected and the light is on, turn it off.
                        elif not latest_occupancy_state and actual_state == 'on':
                            print(f"\nINFO: Occupancy ended. Turning OFF {HA_ENTITY_ID}")
                            set_home_assistant_entity_state('off')

                    return # Exit after processing one full block

        except serial.SerialException as e:
            print(f"Serial error: {e}. Closing port.")
            ser.close()
            break
        except Exception as e:
            print(f"An unexpected error occurred while reading serial: {e}")
            break


def update_plot(frame):
    """Animation function to update the plot."""
    read_serial_data()
    
    line_profile.set_ydata(latest_range_profile)
    line_thresh.set_ydata(latest_cfar_threshold)
    line_targets.set_data(latest_detected_ranges, latest_detected_magnitudes)

    # Update title with occupancy state
    occupancy_str = "YES" if latest_occupancy_state else "NO"
    ax.set_title(f'ESP32 Live Radar Data | Occupancy: {occupancy_str}')

    # Auto-scaling for Y-axis
    profile_for_scaling = latest_range_profile[10:-10]
    profile_for_scaling = profile_for_scaling[~np.isnan(profile_for_scaling)]

    if profile_for_scaling.size > 0:
        y_min = np.min(profile_for_scaling)
        y_max = np.max(profile_for_scaling)
        if np.isfinite(y_min) and np.isfinite(y_max):
            ax.set_ylim(y_min - 5, y_max + 10)
    
    return line_profile, line_thresh, line_targets

# =============================================================================
# --- Main Execution ---
# =============================================================================
if __name__ == '__main__':
    if connect_serial():
        # On startup, ensure the HA entity is off, in case it was left on.
        if HA_ENABLED:
            print("Initializing Home Assistant entity to OFF state.")
            set_home_assistant_entity_state('off')

        ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True)
        plt.tight_layout()
        plt.show()
        
        if ser and ser.is_open:
            print("Plot closed. Closing serial port.")
            # On exit, set the entity to OFF as a safety measure.
            if HA_ENABLED:
                set_home_assistant_entity_state('off')
            ser.close()
    else:
        print("Could not connect to the device. Exiting.")
