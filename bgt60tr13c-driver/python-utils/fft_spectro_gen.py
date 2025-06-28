import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =============================================================================
# --- Configuration ---
# =============================================================================
# !!! IMPORTANT: Change this to the correct serial port for your ESP32 !!!
# Examples: 'COM3' on Windows, '/dev/tty.usbserial-XXXX' on macOS, '/dev/ttyUSB0' on Linux
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'  
BAUD_RATE = 921600

# --- Radar & Plot Parameters ---
# This must match N_RANGE_BINS in your radar_config.h
N_RANGE_BINS = 512
MAX_RANGE_M = 12.87 # Corresponds to R_MAX_M from radar_config.h, adjust if needed

# =============================================================================
# --- Global Variables ---
# =============================================================================
ser = None  # Will hold the serial port object
range_axis = np.linspace(0, MAX_RANGE_M, N_RANGE_BINS)
latest_range_profile = np.full(N_RANGE_BINS, np.nan)
latest_cfar_threshold = np.full(N_RANGE_BINS, np.nan)

# =============================================================================
# --- Plotting Setup ---
# =============================================================================
fig, ax = plt.subplots(figsize=(12, 7))
line_profile, = ax.plot(range_axis, np.zeros(N_RANGE_BINS), 'g-', label='Range Profile (dB)')
line_thresh, = ax.plot(range_axis, np.zeros(N_RANGE_BINS), 'r--', label='CFAR Threshold (dB)')
ax.set_title('ESP32 Live Radar Data')
ax.set_xlabel('Range (m)')
ax.set_ylabel('Magnitude (dB)')
ax.set_xlim(0, MAX_RANGE_M)
ax.set_ylim(-100, 0) # Initial Y-axis limits, will auto-adjust
ax.grid(True)
ax.legend(loc='upper right')

# =============================================================================
# --- Core Logic ---
# =============================================================================
def connect_serial():
    """Tries to connect to the configured serial port."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT}")
        return True
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. Please check the port name and permissions.")
        print(f"Details: {e}")
        return False

def parse_data_line(line):
    """Parses a line of comma-separated values into a numpy array."""
    try:
        # Remove the data label (e.g., "RANGE_PROFILE_DB:") and split by comma
        parts = line.split(':')[1].strip().split(',')
        # Convert each part to a float, handling 'nan' values
        return np.array([float(p) if p != 'nan' else np.nan for p in parts])
    except (ValueError, IndexError) as e:
        print(f"Warning: Could not parse line: {line.strip()}. Error: {e}")
        return None

def read_serial_data():
    """Reads from serial until a complete plot block is received."""
    global latest_range_profile, latest_cfar_threshold
    
    in_plot_block = False
    
    while ser and ser.is_open:
        try:
            line_bytes = ser.readline()
            if not line_bytes:
                # Timeout occurred, just continue
                return

            line = line_bytes.decode('utf-8', errors='ignore').strip()

            if "---PLOT_START---" in line:
                in_plot_block = True
                temp_profile = None
                temp_cfar = None
                continue

            if in_plot_block:
                if line.startswith("RANGE_PROFILE_DB:"):
                    temp_profile = parse_data_line(line)
                elif line.startswith("CFAR_THRESHOLD_DB:"):
                    temp_cfar = parse_data_line(line)
                elif "---PLOT_END---" in line:
                    in_plot_block = False
                    # Update global data only if both parts were received correctly
                    if temp_profile is not None and temp_cfar is not None:
                        if temp_profile.size == N_RANGE_BINS and temp_cfar.size == N_RANGE_BINS:
                            latest_range_profile = temp_profile
                            latest_cfar_threshold = temp_cfar
                            # A complete block was processed, exit the function
                            return 
                        else:
                            print(f"Warning: Data size mismatch. Profile: {temp_profile.size}, CFAR: {temp_cfar.size}. Expected: {N_RANGE_BINS}")
                    else:
                        print("Warning: Incomplete plot block received.")

        except serial.SerialException as e:
            print(f"Serial error: {e}. Closing port.")
            ser.close()
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break


def update_plot(frame):
    """
    This function is called by the animation to update the plot.
    It triggers a serial read and then updates the plot lines.
    """
    # This reads one full data block from the ESP32
    read_serial_data()
    
    # Update the plot data
    line_profile.set_ydata(latest_range_profile)
    line_thresh.set_ydata(latest_cfar_threshold)

    # Auto-adjust Y-axis limits for better visibility, ignoring NaNs
    valid_data = np.concatenate((latest_range_profile, latest_cfar_threshold))
    valid_data = valid_data[~np.isnan(valid_data)]
    
    if valid_data.size > 0:
        y_min = np.min(valid_data) - 10
        y_max = np.max(valid_data) + 10
        ax.set_ylim(y_min, y_max)
    
    return line_profile, line_thresh

# =============================================================================
# --- Main Execution ---
# =============================================================================
if __name__ == '__main__':
    if connect_serial():
        # The 'interval' parameter sets the delay between frames in milliseconds.
        # A smaller interval makes the plot more responsive.
        ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True)
        plt.tight_layout()
        plt.show()
        
        # When the plot window is closed, this code will execute
        if ser and ser.is_open:
            print("Plot closed. Closing serial port.")
            ser.close()
    else:
        print("Could not connect to the device. Exiting.")
