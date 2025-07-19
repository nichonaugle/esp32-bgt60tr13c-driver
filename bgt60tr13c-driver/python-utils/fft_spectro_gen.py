import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.constants import c
import time
from collections import deque

# =============================================================================
# Algorithm & Tuning Parameters
# =============================================================================
# --- General ---
HIGH_RES_FFT_LEN = 1024
CHIRPS_PER_FRAME = 64

# --- Presence Detection (Phase Analysis) ---
PRESENCE_HISTORY_LEN = 8
PRESENCE_MAG_THRESHOLD = 500
PRESENCE_PHASE_STD_THRESHOLD = 0.32

# --- Presence Confirmation Logic ---
# This logic prevents false positives by requiring detections to be consistent
# over time and space before confirming a "presence".
MIN_HITS_FOR_CONFIRMATION = 3      # How many detections in a row are needed to confirm presence.
MAX_FRAME_GAP_ALLOWED = 2          # How many frames can be missed before the hit counter resets.
MAX_RANGE_DIFFERENCE_METERS = 1.5  # Max distance (m) a detection can move between frames to be part of the same event.

# =============================================================================
# Radar Parameters and Setup
# =============================================================================
f_low = 59479900000
f_high = 60436900000
f_center = (f_low + f_high) / 2
f_bandwidth = abs(f_high - f_low)
sample_rate = 2352941
samples_per_chirp = 128
Tc = samples_per_chirp / sample_rate
PRT = 0.000690325

r_max = (sample_rate * c * Tc) / (4 * f_bandwidth)
range_axis = np.linspace(0, r_max, HIGH_RES_FFT_LEN // 2)

# --- Serial and Plotting Setup ---
try:
    ser = serial.Serial('/dev/tty.SLAB_USBtoUART5', 921600, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

plt.ion()
fig, ax_presence = plt.subplots(figsize=(12, 6))

ax_presence.set_title("Presence Detection via Phase Fluctuation Analysis")
ax_presence.set_xlabel("Range (m)")
ax_presence.set_ylabel("Phase Standard Deviation (Radians)")
ax_presence.set_xlim([0, r_max])
ax_presence.grid(True)
line_phase_std, = ax_presence.plot(range_axis, np.zeros_like(range_axis), c='cyan', lw=2, label='Phase Std Dev')
line_phase_thresh = ax_presence.axhline(PRESENCE_PHASE_STD_THRESHOLD, color='r', ls='--', lw=1.5, label='Detection Threshold')
presence_detections_scatter = ax_presence.scatter([], [], c='orange', s=120, zorder=10, label='Raw Detection')
# Add a new text element for the confirmation status
confirmation_text = ax_presence.text(0.5, 0.95, 'STATUS: NO PRESENCE', ha='center', va='top',
                                     transform=ax_presence.transAxes, fontsize=14,
                                     bbox=dict(boxstyle='round,pad=0.3', fc='lightgray', alpha=0.5))
ax_presence.legend(loc="upper right")
fig.tight_layout()

# --- Processing Variables ---
presence_history = deque(maxlen=PRESENCE_HISTORY_LEN)
frame_counter = 0

# State variables for the new confirmation logic
confirmation_hits = 0
last_detection_frame = -1
last_detection_range = 0.0
presence_confirmed = False

# =============================================================================
# Main Processing Loop
# =============================================================================
print("Starting radar processing loop... Press Ctrl+C to stop.")
while True:
    try:
        line = ser.readline().decode().strip()
        if not line.startswith('Frame'):
            continue
        
        frame_counter += 1
        data_str = line.split(':', 1)[1].strip()
        data = [float(x) for x in data_str.strip('[]').split(',') if x]
        
        if not data or len(data) < CHIRPS_PER_FRAME * samples_per_chirp:
            continue

        raw_data = np.array(data, dtype=np.float32).reshape((CHIRPS_PER_FRAME, samples_per_chirp))

        # --- Standard Processing Steps ---
        range_ffts = fft(raw_data * np.hanning(samples_per_chirp), n=HIGH_RES_FFT_LEN, axis=1)
        coherently_integrated_profile = np.mean(range_ffts[:, :HIGH_RES_FFT_LEN // 2], axis=0)
        presence_history.append(coherently_integrated_profile)

        detected_indices = [] # Default to no detections
        if len(presence_history) == PRESENCE_HISTORY_LEN:
            history_matrix = np.array(presence_history)
            unwrapped_phases = np.unwrap(np.angle(history_matrix), axis=0)
            phase_std_dev = np.std(unwrapped_phases, axis=0)
            avg_magnitude = np.mean(np.abs(history_matrix), axis=0)
            
            detected_indices = np.where(
                (phase_std_dev > PRESENCE_PHASE_STD_THRESHOLD) &
                (avg_magnitude > PRESENCE_MAG_THRESHOLD)
            )[0]
            
            line_phase_std.set_ydata(phase_std_dev)
            max_y = np.max(phase_std_dev) * 1.2
            ax_presence.set_ylim([0, max(max_y, PRESENCE_PHASE_STD_THRESHOLD * 1.5)])

        # --- NEW: PRESENCE CONFIRMATION STATE MACHINE ---
        is_detection_in_frame = len(detected_indices) > 0

        if is_detection_in_frame:
            # Find the range of the strongest detection in this frame
            strongest_detection_idx = detected_indices[np.argmax(phase_std_dev[detected_indices])]
            strongest_detection_range = range_axis[strongest_detection_idx]

            if confirmation_hits == 0:
                # This is the first hit of a potential new sequence
                confirmation_hits = 1
                last_detection_range = strongest_detection_range
            else:
                # This is a potential continuation of a sequence
                frame_gap = frame_counter - last_detection_frame
                range_diff = abs(strongest_detection_range - last_detection_range)

                if frame_gap <= (MAX_FRAME_GAP_ALLOWED + 1) and range_diff <= MAX_RANGE_DIFFERENCE_METERS:
                    # The detection is close in time and space: it's a valid continuation
                    confirmation_hits += 1
                    last_detection_range = strongest_detection_range # Update to the latest position
                else:
                    # The detection is too far in time or space; reset and start a new sequence
                    confirmation_hits = 1
                    last_detection_range = strongest_detection_range
            
            last_detection_frame = frame_counter # Always update the frame of the last raw detection
        
        else: # No detection in this frame
            frame_gap = frame_counter - last_detection_frame
            if frame_gap > MAX_FRAME_GAP_ALLOWED:
                # The allowed gap has been exceeded, reset everything
                confirmation_hits = 0
                presence_confirmed = False

        # Finally, update the official confirmation status
        if confirmation_hits >= MIN_HITS_FOR_CONFIRMATION:
            presence_confirmed = True
        
        # --- Update Plot ---
        if is_detection_in_frame:
            detected_ranges = range_axis[detected_indices]
            presence_detections_scatter.set_offsets(np.c_[detected_ranges, phase_std_dev[detected_indices]])
        else:
            presence_detections_scatter.set_offsets(np.empty((0, 2)))

        if presence_confirmed:
            confirmation_text.set_text('STATUS: ✅ PRESENCE CONFIRMED')
            confirmation_text.set_bbox(dict(boxstyle='round,pad=0.3', fc='lightgreen', alpha=0.8))
        else:
            confirmation_text.set_text(f'STATUS: NO PRESENCE (Hits: {confirmation_hits})')
            confirmation_text.set_bbox(dict(boxstyle='round,pad=0.3', fc='lightgray', alpha=0.5))

        # --- Print Final Status to Console ---
        status_text = f"Status -> Confirmed: {'✅ YES' if presence_confirmed else 'No '} | Hits: {confirmation_hits}"
        print(f"\r{status_text.ljust(50)}", end="")

        fig.canvas.draw_idle()
        plt.pause(0.001)

    except KeyboardInterrupt:
        print("\nStopping script.")
        break
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        time.sleep(1)
        continue

# =============================================================================
# Cleanup
# =============================================================================
ser.close()
plt.ioff()
plt.show()
print("Serial port closed. Program finished.")
