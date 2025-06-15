import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import re

# --- Serial Configuration ---
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART'  # Change to your ESP32's serial port
BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.2  # Increased timeout for larger data

# --- Plotting Setup ---
plt.ion()  # Interactive mode
fig, ax = plt.subplots(figsize=(14, 8))  # Wider plot for higher resolution
line_signal, = ax.plot([], [], 'b-', linewidth=1.5, label='Range Profile')
line_threshold, = ax.plot([], [], 'r--', linewidth=2, label='CFAR Threshold')
line_detections, = ax.plot([], [], 'go', markersize=8, label='Detections')

ax.set_xlabel('Range (m)')
ax.set_ylabel('Magnitude (dB)')
ax.set_title('Real-Time Range Profile & CFAR Detection (4x Zero-Padded)')
ax.legend()
ax.grid(True, alpha=0.3)
# Don't set fixed limits - let it auto-scale based on data
ax.set_ylim(0, 120)  # Typical dB range

plt.tight_layout()
plt.show(block=False)

def parse_range_profile_data(ser):
    """Parse range profile data from ESP32 UART output"""
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line == "RANGE_PROFILE_START":
                # Read the data lines
                range_data = None
                cfar_data = None
                range_bins = None
                detections = []
                
                # Parse data lines - increased timeout for larger data
                for _ in range(15):  # More lines to search through
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith("RANGE_DATA:"):
                        data_str = line.replace("RANGE_DATA:", "")
                        try:
                            # Handle larger data arrays (512 instead of 128 values)
                            values = data_str.split(',')
                            range_data = np.array([float(x) for x in values if x.strip()])
                            print(f"Parsed {len(range_data)} range data points")
                        except ValueError as e:
                            print(f"Error parsing range data: {e}")
                            continue
                        
                    elif line.startswith("CFAR_DATA:"):
                        data_str = line.replace("CFAR_DATA:", "")
                        try:
                            cfar_values = []
                            values = data_str.split(',')
                            for x in values:
                                x = x.strip()
                                if x == 'nan' or x == '':
                                    cfar_values.append(np.nan)
                                else:
                                    cfar_values.append(float(x))
                            cfar_data = np.array(cfar_values)
                            print(f"Parsed {len(cfar_data)} CFAR data points")
                        except ValueError as e:
                            print(f"Error parsing CFAR data: {e}")
                            continue
                        
                    elif line.startswith("RANGE_BINS:"):
                        data_str = line.replace("RANGE_BINS:", "")
                        try:
                            values = data_str.split(',')
                            range_bins = np.array([float(x) for x in values if x.strip()])
                            print(f"Parsed {len(range_bins)} range bins")
                        except ValueError as e:
                            print(f"Error parsing range bins: {e}")
                            continue
                        
                    elif "Target detected at range:" in line:
                        # Parse detection: "Target detected at range: 0.127 m (Magnitude: 85.20 dB, Threshold: 78.10 dB)"
                        # Updated regex to handle 3 decimal places for range
                        match = re.search(r'range: ([\d.]+) m.*Magnitude: ([\d.]+) dB', line)
                        if match:
                            det_range = float(match.group(1))
                            det_magnitude = float(match.group(2))
                            detections.append((det_range, det_magnitude))
                            print(f"✓ Detection parsed: {det_range:.3f}m at {det_magnitude:.1f}dB")
                        else:
                            print(f"✗ Failed to parse detection line: {line}")
                    
                    elif line == "RANGE_PROFILE_END":
                        break
                
                # Validate data consistency
                if range_data is not None and cfar_data is not None and range_bins is not None:
                    if len(range_data) == len(cfar_data) == len(range_bins):
                        max_range = np.max(range_bins)
                        min_range = np.min(range_bins)
                        print(f"✓ Complete dataset: {len(range_data)} points, range {min_range:.3f}m to {max_range:.3f}m")
                        return range_bins, range_data, cfar_data, detections
                    else:
                        print(f"✗ Data length mismatch: range={len(range_data)}, cfar={len(cfar_data)}, bins={len(range_bins)}")
                        return None, None, None, None
                else:
                    print("✗ Incomplete data received")
                    return None, None, None, None
                    
    except Exception as e:
        print(f"Error parsing data: {e}")
        return None, None, None, None

def update_plot(range_bins, range_data, cfar_data, detections):
    """Update the matplotlib plot with new data"""
    try:
        # Update range profile line
        line_signal.set_data(range_bins, range_data)
        
        # Update CFAR threshold line (only valid values)
        valid_mask = ~np.isnan(cfar_data)
        if np.any(valid_mask):
            line_threshold.set_data(range_bins[valid_mask], cfar_data[valid_mask])
        else:
            line_threshold.set_data([], [])
        
        # Update detection markers
        if detections:
            det_ranges, det_mags = zip(*detections)
            line_detections.set_data(det_ranges, det_mags)
        else:
            line_detections.set_data([], [])
        
        # Auto-scale X axis based on actual data range
        if len(range_bins) > 0:
            max_range = np.max(range_bins)
            ax.set_xlim(0, max_range)
            
        # Auto-scale Y axis
        if len(range_data) > 0:
            valid_range_data = range_data[~np.isnan(range_data)]
            valid_cfar_data = cfar_data[~np.isnan(cfar_data)]
            
            if len(valid_range_data) > 0:
                min_val = min(np.min(valid_range_data), np.min(valid_cfar_data) if len(valid_cfar_data) > 0 else np.min(valid_range_data))
                max_val = max(np.max(valid_range_data), np.max(valid_cfar_data) if len(valid_cfar_data) > 0 else np.max(valid_range_data))
                ax.set_ylim(min_val - 5, max_val + 5)
        
        # Update title with data info
        max_range_str = f"{np.max(range_bins):.2f}m" if len(range_bins) > 0 else "unknown"
        ax.set_title(f'Real-Time Range Profile & CFAR Detection ({len(range_data)} bins, 0-{max_range_str})')
        
        # Refresh plot
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        
    except Exception as e:
        print(f"Error updating plot: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main function to read serial data and plot"""
    # Connect to serial port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        print("Timeout increased to handle larger data packets (512 vs 128 bins)")
        time.sleep(1)  # Allow ESP32 to initialize
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return
    
    print("Waiting for range profile data...")
    print("Expected: 512 range bins (4x zero-padded FFT)")
    print("Close the plot window to exit.")
    
    frame_count = 0
    try:
        while plt.fignum_exists(fig.number):
            # Parse new data
            range_bins, range_data, cfar_data, detections = parse_range_profile_data(ser)
            
            if range_bins is not None:
                frame_count += 1
                detection_count = len(detections)
                print(f"Frame {frame_count}: Updated plot with {len(range_bins)} bins, {detection_count} detections")
                if detection_count > 0:
                    print(f"  Detections: {[(f'{r:.3f}m', f'{m:.1f}dB') for r, m in detections]}")
                update_plot(range_bins, range_data, cfar_data, detections)
            
            plt.pause(0.01)  # Small pause to prevent high CPU usage
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")
        plt.ioff()
        print("Done")

if __name__ == "__main__":
    main()