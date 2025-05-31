import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from scipy import signal
from scipy.signal import windows
from scipy import constants
import matplotlib.pyplot as plt
import time
import serial
import re
import traceback
import tkinter as tk
from tkinter import ttk

# Radar Configuration
NUM_CHIRPS_PER_FRAME = 32
NUM_SAMPLES_PER_CHIRP = 128
NUM_RX_ANTENNAS = 3

START_FREQUENCY_HZ = 60000000000  # Updated: Was 58e9, now 60e9 (60 GHz)
END_FREQUENCY_HZ = 62000000000    # Updated: Was 59e9, now 62e9 (60 GHz + 2 GHz bandwidth)
SAMPLE_RATE_HZ = 2000000          # Updated: Was 200000, now 2000000 (2 MSps)
CHIRP_REPETITION_TIME_S = 0.0001935 # Updated: Was 0.00059111, now ~0.0001935s

# Hardcoded Settings
SERIAL_PORT = '/dev/tty.SLAB_USBtoUART2'
BAUD_RATE = 921600
PROCESSING_FRAMERATE_HZ = 10

EXPECTED_SAMPLES_IN_FLAT_ARRAY = NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS

# Constants for signal handling
BINS_TO_SKIP_FOR_PEAK_DETECTION = 15
BINS_TO_IGNORE_FOR_PLOT_YLIM = 15

# Initial Filtering Configuration
CALIBRATION_KEY = 'c'
INITIAL_THRESHOLD_1D_PROFILE_LINEAR = 0.05
INITIAL_THRESHOLD_2D_MAG_LINEAR = 0.1
FILTERING_FLOOR_1D_PROFILE_LINEAR = 0.0
FILTERING_FLOOR_2D_MAG_LINEAR = 1e-9

# AppState Class
class AppState:
    def __init__(self):
        self.calibration_requested = False
        self.filtering_enabled = False
        self.baseline_1d_profiles = None
        self.baseline_2d_fft_abs = None
        self.baseline_rd_maps_abs = None
        
        self.threshold_1d_profile = INITIAL_THRESHOLD_1D_PROFILE_LINEAR
        self.threshold_2d_mag = INITIAL_THRESHOLD_2D_MAG_LINEAR
        
        self.gui_is_active = False
        
        print(f"Press '{CALIBRATION_KEY}' in the plot window to calibrate baseline noise.")
        print("A separate window for threshold controls will also appear.")

# Threshold GUI Setup
def setup_threshold_gui(app_state_obj):
    root = tk.Tk()
    root.title("Filtering Thresholds")
    root.geometry("300x180") 

    ttk.Label(root, text="1D Profile Threshold:").pack(pady=(10,0))
    val_1d_sv = tk.StringVar(value=f"{app_state_obj.threshold_1d_profile:.3f}")

    def on_1d_threshold_change(*args): 
        try:
            value = float(val_1d_sv.get())
            app_state_obj.threshold_1d_profile = value
        except ValueError:
            val_1d_sv.set(f"{app_state_obj.threshold_1d_profile:.3f}")

    val_1d_sv.trace_add("write", on_1d_threshold_change)

    spinbox_1d = ttk.Spinbox(root, from_=0.0, to=1.0, increment=0.01, width=10,
                             textvariable=val_1d_sv, format="%.3f") 
    spinbox_1d.pack(pady=5)

    ttk.Label(root, text="2D Magnitude Threshold:").pack(pady=(10,0))
    val_2d_sv = tk.StringVar(value=f"{app_state_obj.threshold_2d_mag:.3f}")

    def on_2d_threshold_change(*args):
        try:
            value = float(val_2d_sv.get())
            app_state_obj.threshold_2d_mag = value
        except ValueError:
            val_2d_sv.set(f"{app_state_obj.threshold_2d_mag:.3f}")

    val_2d_sv.trace_add("write", on_2d_threshold_change)

    spinbox_2d = ttk.Spinbox(root, from_=0.0, to=2.0, increment=0.01, width=10,
                             textvariable=val_2d_sv, format="%.3f")
    spinbox_2d.pack(pady=5)
    
    def on_closing_gui():
        app_state_obj.gui_is_active = False

    root.protocol("WM_DELETE_WINDOW", on_closing_gui)
    app_state_obj.gui_is_active = True
    return root

# RadarConfig Class
class RadarConfig:
    def __init__(self, num_chirps, num_samples, start_freq, end_freq, proc_framerate_hz, sample_rate_hz, chirp_rep_time_s):
        self.num_chirps_per_frame = num_chirps
        self.num_samples_per_chirp = num_samples
        self.start_frequency_Hz = start_freq
        self.end_frequency_Hz = end_freq
        self.frame_repetition_time_s = 1.0 / proc_framerate_hz if proc_framerate_hz > 0 else 0.01
        self.sample_rate_Hz = sample_rate_hz
        self.rx_mask = (1 << NUM_RX_ANTENNAS) - 1
        self.tx_mask = 1
        self.tx_power_level = 31
        self.if_gain_dB = 33
        self.chirp_repetition_time_s = chirp_rep_time_s

        if (start_freq + end_freq) > 0:
            self.center_frequency_Hz = (start_freq + end_freq) / 2.0
        else:
            self.center_frequency_Hz = (start_freq + end_freq) / 2.0 
            print("Warning: Radar frequencies are zero or not set properly. Center frequency is zero.")
        
        if self.center_frequency_Hz > 0:
            self.wavelength_m = constants.c / self.center_frequency_Hz
        else:
            self.wavelength_m = 0 
            print("Warning: Center frequency is zero. Wavelength cannot be calculated.")

# FFT Spectrum Function
def fft_spectrum(data_matrix, window_vector):
    windowed_data = data_matrix * window_vector
    fft_size = data_matrix.shape[1] * 2 
    complex_fft_result = np.fft.fft(windowed_data, n=fft_size, axis=1)
    return complex_fft_result[:, :data_matrix.shape[1]] 

# DistanceFFT_Algo Class
class DistanceFFT_Algo:
    def __init__(self, config: RadarConfig):
        self._numchirps = config.num_chirps_per_frame
        self.chirpsamples = config.num_samples_per_chirp
        self._range_window = windows.blackmanharris(self.chirpsamples).reshape(1, self.chirpsamples)
        
        start_frequency_Hz = config.start_frequency_Hz
        end_frequency_Hz = config.end_frequency_Hz
        bandwidth_hz = abs(end_frequency_Hz - start_frequency_Hz)
        fft_size_for_bin_calc = self.chirpsamples * 2 
        
        if bandwidth_hz > 0:
            self._range_bin_length = (constants.c) / (2 * bandwidth_hz * (fft_size_for_bin_calc / self.chirpsamples))
        else:
            self._range_bin_length = 0
            print("Warning: Bandwidth is zero. Range bin length cannot be calculated.")
        
        self._config = config
        if self._numchirps > 0:
            self._doppler_window = windows.hann(self._numchirps).reshape(self._numchirps, 1)
            if config.center_frequency_Hz > 0 and config.chirp_repetition_time_s > 0:
                doppler_freqs_shifted = np.fft.fftshift(np.fft.fftfreq(self._numchirps, d=config.chirp_repetition_time_s))
                self._velocity_axis_mps = doppler_freqs_shifted * config.wavelength_m / 2.0
                max_doppler_freq = 1.0 / (2.0 * config.chirp_repetition_time_s) 
                self.max_unambiguous_velocity_mps = (max_doppler_freq * config.wavelength_m) / 2.0
                print(f"Max unambiguous velocity: +/- {self.max_unambiguous_velocity_mps:.2f} m/s")
            else:
                self._velocity_axis_mps = np.zeros(self._numchirps)
                self.max_unambiguous_velocity_mps = 0
                print("Warning: Cannot calculate velocity axis due to zero center frequency or chirp repetition time.")
        else:
            self._doppler_window = np.array([]) 
            self._velocity_axis_mps = np.array([])
            self.max_unambiguous_velocity_mps = 0
            print("Warning: Number of chirps is zero. Doppler processing disabled.")

    def _compute_range_doppler_map(self, range_fft_complex_data):
        if self._numchirps == 0 or range_fft_complex_data.shape[0] != self._numchirps:
            num_doppler_bins = self._numchirps if self._numchirps > 0 else 1
            num_range_bins = range_fft_complex_data.shape[1]
            return np.zeros((num_doppler_bins, num_range_bins))

        windowed_data_for_doppler = range_fft_complex_data * self._doppler_window
        range_doppler_fft_complex = np.fft.fft(windowed_data_for_doppler, axis=0)
        range_doppler_fft_shifted = np.fft.fftshift(range_doppler_fft_complex, axes=0)
        return np.abs(range_doppler_fft_shifted)

    def compute_distance_and_profile(self, data):
        range_fft_complex = fft_spectrum(data, self._range_window) 
        fft_spec_abs_per_chirp = np.abs(range_fft_complex) 

        if self._numchirps > 0:
            data_plot_1d = np.divide(fft_spec_abs_per_chirp.sum(axis=0), self._numchirps)
        else:
            data_plot_1d = np.zeros(data.shape[1]) 
        
        skip = BINS_TO_SKIP_FOR_PEAK_DETECTION
        peak_idx_in_slice = np.argmax(data_plot_1d[skip:]) if len(data_plot_1d[skip:]) > 0 else 0
        actual_peak_idx = peak_idx_in_slice + skip
        dist = self._range_bin_length * actual_peak_idx
        
        if self._numchirps > 0:
            range_doppler_map_abs = self._compute_range_doppler_map(range_fft_complex)
        else:
            num_doppler_bins = self._numchirps if self._numchirps > 0 else 1 
            num_range_bins = self.chirpsamples
            range_doppler_map_abs = np.zeros((num_doppler_bins, num_range_bins))
        return dist, data_plot_1d, fft_spec_abs_per_chirp, range_doppler_map_abs

# Draw Class
class Draw: 
    def __init__(self, config: RadarConfig, max_range_m_calc, num_ant, range_bin_length, velocity_axis_mps, app_state: AppState):
        self._num_ant = num_ant
        self.config = config 
        self.chirpsamples = config.num_samples_per_chirp
        self.app_state = app_state 
        
        self._line_plots_orig = [] 
        self._image_plots_rc_orig = [] 
        self._colorbars_rc_orig = []   
        self._range_doppler_plots_orig = [] 
        self._range_doppler_colorbars_orig = [] 

        self._line_plots_filt = []
        self._image_plots_rc_filt = []
        self._colorbars_rc_filt = []
        self._range_doppler_plots_filt = []
        self._range_doppler_colorbars_filt = []

        self._fig, self._axs = plt.subplots(nrows=3, ncols=self._num_ant * 2, 
                                           figsize=(7 * self._num_ant * 2, 5 * 3), 
                                           squeeze=False) 
        self._fig.canvas.manager.set_window_title(
            f"Radar FFT: Left=Original, Right=Filtered (Press '{CALIBRATION_KEY}' to calibrate)")
        
        self.bins_to_ignore_for_1d_plot_ylim = BINS_TO_IGNORE_FOR_PLOT_YLIM 
        self._range_bin_length = range_bin_length
        self._velocity_axis_mps = velocity_axis_mps 

        self._dist_points = np.array([i * self._range_bin_length for i in range(self.chirpsamples)])
        self._chirp_indices = np.arange(self.config.num_chirps_per_frame)

        self._fig.canvas.mpl_connect('close_event', self.close)
        self._fig.canvas.mpl_connect('key_press_event', self._on_key_press) 
        self._is_window_open = True
        plt.ion() 

    def _on_key_press(self, event):
        if event.key == CALIBRATION_KEY:
            self.app_state.calibration_requested = True 

    def _get_1d_plot_ylimits(self, data_all_antennas_1d_profiles, is_filtered=False):
        min_val_overall = float('inf')
        max_val_overall = float('-inf')
        for data_one_antenna in data_all_antennas_1d_profiles:
            if not isinstance(data_one_antenna, np.ndarray) or data_one_antenna.size == 0:
                continue
            
            data_for_calc = data_one_antenna
            if not is_filtered and data_one_antenna.size > self.bins_to_ignore_for_1d_plot_ylim:
                data_for_calc = data_one_antenna[self.bins_to_ignore_for_1d_plot_ylim:]
            
            if data_for_calc.size > 0: 
                current_max = np.max(data_for_calc)
                if current_max > max_val_overall: max_val_overall = current_max
                current_min = np.min(data_for_calc) 
                if current_min < min_val_overall: min_val_overall = current_min
            
        if max_val_overall == float('-inf'): max_val_overall = 1.0 
        if min_val_overall == float('inf'): min_val_overall = 0.0
        if max_val_overall <= min_val_overall: 
            max_val_overall = min_val_overall + (0.01 if is_filtered else 0.1) 
        
        if is_filtered:
             min_val_overall = min(min_val_overall, -0.01) 
        
        return min_val_overall, max_val_overall * (1.02 if is_filtered else 1.05) 


    def _draw_first_time(self, data_1d_orig, data_2d_rc_orig, data_rd_orig,
                         data_1d_filt, data_2d_rc_filt, data_rd_filt):
        
        min_y_1d_orig_all_ant, max_y_1d_orig_all_ant = self._get_1d_plot_ylimits(data_1d_orig)
        min_y_1d_filt_all_ant, max_y_1d_filt_all_ant = self._get_1d_plot_ylimits(data_1d_filt, is_filtered=True)

        dist_start = self._dist_points[0] if len(self._dist_points) > 0 else 0
        dist_end = self._dist_points[-1] if len(self._dist_points) > 0 else 1 
        vel_start = self._velocity_axis_mps[0] if len(self._velocity_axis_mps) > 0 else -1
        vel_end = self._velocity_axis_mps[-1] if len(self._velocity_axis_mps) > 0 else 1
        
        for i_ant in range(self._num_ant):
            col_orig = i_ant * 2 
            col_filt = i_ant * 2 + 1 

            ax_orig_1d = self._axs[0, col_orig] 
            line_o, = ax_orig_1d.plot(self._dist_points[:len(data_1d_orig[i_ant])], data_1d_orig[i_ant])
            ax_orig_1d.set_ylim(min_y_1d_orig_all_ant, max_y_1d_orig_all_ant)
            self._line_plots_orig.append(line_o)
            ax_orig_1d.set_xlabel("Distance (m)"); ax_orig_1d.set_ylabel("Summed FFT Mag")
            ax_orig_1d.set_title(f"Ant {i_ant} - Original Range")

            ax_filt_1d = self._axs[0, col_filt]
            line_f, = ax_filt_1d.plot(self._dist_points[:len(data_1d_filt[i_ant])], data_1d_filt[i_ant])
            ax_filt_1d.set_ylim(min_y_1d_filt_all_ant, max_y_1d_filt_all_ant)
            self._line_plots_filt.append(line_f)
            ax_filt_1d.set_xlabel("Distance (m)"); ax_filt_1d.set_ylabel("Filtered FFT Mag")
            ax_filt_1d.set_title(f"Ant {i_ant} - Filtered Range")

            ax_orig_rc = self._axs[1, col_orig]
            data_2d_abs_o = data_2d_rc_orig[i_ant]; data_2d_db_o = 20 * np.log10(data_2d_abs_o + 1e-9) 
            vmin_o, vmax_o = np.percentile(data_2d_db_o, [5, 95]) if data_2d_db_o.size > 0 else (-60, 0) 
            if vmin_o == vmax_o: vmax_o = vmin_o + 1 
            img_o = ax_orig_rc.imshow(data_2d_db_o, aspect='auto', cmap='viridis', origin='lower', vmin=vmin_o, vmax=vmax_o,
                                   extent=[dist_start, dist_end, self._chirp_indices[0] - 0.5 if len(self._chirp_indices)>0 else -0.5, self._chirp_indices[-1] + 0.5 if len(self._chirp_indices)>0 else 0.5])
            self._image_plots_rc_orig.append(img_o)
            cb_o = self._fig.colorbar(img_o, ax=ax_orig_rc, label='Mag (dB)'); self._colorbars_rc_orig.append(cb_o)
            ax_orig_rc.set_xlabel("Distance (m)"); ax_orig_rc.set_ylabel("Chirp Index")
            ax_orig_rc.set_title(f"Ant {i_ant} - Orig Range/Chirp")

            ax_filt_rc = self._axs[1, col_filt]
            data_2d_abs_f = data_2d_rc_filt[i_ant]; data_2d_db_f = 20 * np.log10(data_2d_abs_f + FILTERING_FLOOR_2D_MAG_LINEAR) 
            vmin_f, vmax_f = np.percentile(data_2d_db_f, [5, 98]) if data_2d_db_f.size > 0 else (-70, -10)
            if vmin_f == vmax_f: vmax_f = vmin_f + 10
            img_f = ax_filt_rc.imshow(data_2d_db_f, aspect='auto', cmap='viridis', origin='lower', vmin=vmin_f, vmax=vmax_f,
                                    extent=[dist_start, dist_end, self._chirp_indices[0] - 0.5 if len(self._chirp_indices)>0 else -0.5, self._chirp_indices[-1] + 0.5 if len(self._chirp_indices)>0 else 0.5])
            self._image_plots_rc_filt.append(img_f)
            cb_f = self._fig.colorbar(img_f, ax=ax_filt_rc, label='Filt. Mag (dB)'); self._colorbars_rc_filt.append(cb_f)
            ax_filt_rc.set_xlabel("Distance (m)"); ax_filt_rc.set_ylabel("Chirp Index")
            ax_filt_rc.set_title(f"Ant {i_ant} - Filt Range/Chirp")

            ax_orig_rd = self._axs[2, col_orig]
            rd_map_abs_o = data_rd_orig[i_ant]; rd_map_db_o = 20 * np.log10(rd_map_abs_o + 1e-9)
            vmin_rd_o, vmax_rd_o = np.percentile(rd_map_db_o, [15, 98]) if rd_map_db_o.size > 0 else (-80, -20) 
            if vmin_rd_o == vmax_rd_o: vmax_rd_o = vmin_rd_o + 10
            rd_img_o = ax_orig_rd.imshow(rd_map_db_o, aspect='auto', cmap='jet', origin='lower', vmin=vmin_rd_o, vmax=vmax_rd_o, extent=[dist_start, dist_end, vel_start, vel_end])
            self._range_doppler_plots_orig.append(rd_img_o)
            rd_cb_o = self._fig.colorbar(rd_img_o, ax=ax_orig_rd, label='Mag (dB)'); self._range_doppler_colorbars_orig.append(rd_cb_o)
            ax_orig_rd.set_xlabel("Distance (m)"); ax_orig_rd.set_ylabel("Velocity (m/s)")
            ax_orig_rd.set_title(f"Ant {i_ant} - Original R-D")

            ax_filt_rd = self._axs[2, col_filt]
            rd_map_abs_f = data_rd_filt[i_ant]; rd_map_db_f = 20 * np.log10(rd_map_abs_f + FILTERING_FLOOR_2D_MAG_LINEAR)
            vmin_rd_f, vmax_rd_f = np.percentile(rd_map_db_f, [15, 99]) if rd_map_db_f.size > 0 else (-90, -30)
            if vmin_rd_f == vmax_rd_f: vmax_rd_f = vmin_rd_f + 10
            rd_img_f = ax_filt_rd.imshow(rd_map_db_f, aspect='auto', cmap='jet', origin='lower', vmin=vmin_rd_f, vmax=vmax_rd_f, extent=[dist_start, dist_end, vel_start, vel_end])
            self._range_doppler_plots_filt.append(rd_img_f)
            rd_cb_f = self._fig.colorbar(rd_img_f, ax=ax_filt_rd, label='Filt. Mag (dB)'); self._range_doppler_colorbars_filt.append(rd_cb_f)
            ax_filt_rd.set_xlabel("Distance (m)"); ax_filt_rd.set_ylabel("Velocity (m/s)")
            ax_filt_rd.set_title(f"Ant {i_ant} - Filtered R-D")

        self._fig.tight_layout(pad=1.5, h_pad=2.5, w_pad=2.5) 
        plt.show() 

    def _update_plots(self, data_1d_orig, data_2d_rc_orig, data_rd_orig,
                      data_1d_filt, data_2d_rc_filt, data_rd_filt):
        
        min_y_orig_all, max_y_orig_all = self._get_1d_plot_ylimits(data_1d_orig)
        min_y_filt_all, max_y_filt_all = self._get_1d_plot_ylimits(data_1d_filt, is_filtered=True)

        for i_ant in range(self._num_ant):
            col_orig = i_ant * 2
            col_filt = i_ant * 2 + 1

            if i_ant < len(self._line_plots_orig): self._line_plots_orig[i_ant].set_ydata(data_1d_orig[i_ant]); self._axs[0, col_orig].set_ylim(min_y_orig_all, max_y_orig_all)
            if i_ant < len(self._line_plots_filt): self._line_plots_filt[i_ant].set_ydata(data_1d_filt[i_ant]); self._axs[0, col_filt].set_ylim(min_y_filt_all, max_y_filt_all)
            
            if i_ant < len(self._image_plots_rc_orig):
                data_db_o = 20 * np.log10(data_2d_rc_orig[i_ant] + 1e-9)
                self._image_plots_rc_orig[i_ant].set_data(data_db_o)
                vmin_o, vmax_o = np.percentile(data_db_o, [5, 95]) if data_db_o.size > 0 else (-60,0); 
                self._image_plots_rc_orig[i_ant].set_clim(vmin=vmin_o, vmax=vmax_o if vmin_o != vmax_o else vmin_o +1)
        
            if i_ant < len(self._image_plots_rc_filt):
                data_db_f = 20 * np.log10(data_2d_rc_filt[i_ant] + FILTERING_FLOOR_2D_MAG_LINEAR)
                self._image_plots_rc_filt[i_ant].set_data(data_db_f)
                vmin_f, vmax_f = np.percentile(data_db_f, [5, 98]) if data_db_f.size > 0 else (-70, -10); 
                self._image_plots_rc_filt[i_ant].set_clim(vmin=vmin_f, vmax=vmax_f if vmin_f != vmax_f else vmin_f+10)
        
            if i_ant < len(self._range_doppler_plots_orig):
                rd_db_o = 20 * np.log10(data_rd_orig[i_ant] + 1e-9)
                self._range_doppler_plots_orig[i_ant].set_data(rd_db_o)
                vmin_rd_o, vmax_rd_o = np.percentile(rd_db_o, [15, 98]) if rd_db_o.size > 0 else (-80,-20); 
                self._range_doppler_plots_orig[i_ant].set_clim(vmin=vmin_rd_o, vmax=vmax_rd_o if vmin_rd_o != vmax_rd_o else vmin_rd_o+10)

            if i_ant < len(self._range_doppler_plots_filt):
                rd_db_f = 20 * np.log10(data_rd_filt[i_ant] + FILTERING_FLOOR_2D_MAG_LINEAR)
                self._range_doppler_plots_filt[i_ant].set_data(rd_db_f)
                vmin_rd_f, vmax_rd_f = np.percentile(rd_db_f, [15, 99]) if rd_db_f.size > 0 else (-90, -30); 
                self._range_doppler_plots_filt[i_ant].set_clim(vmin=vmin_rd_f, vmax=vmax_rd_f if vmin_rd_f != vmax_rd_f else vmin_rd_f+10)

    def draw(self, data_1d_orig, data_2d_rc_orig, data_rd_orig,
             data_1d_filt, data_2d_rc_filt, data_rd_filt):
        if self._is_window_open:
            if not self._line_plots_orig: 
                self._draw_first_time(data_1d_orig, data_2d_rc_orig, data_rd_orig,
                                      data_1d_filt, data_2d_rc_filt, data_rd_filt)
            else: 
                self._update_plots(data_1d_orig, data_2d_rc_orig, data_rd_orig,
                                   data_1d_filt, data_2d_rc_filt, data_rd_filt)
            
            self._fig.canvas.draw_idle()  
            self._fig.canvas.flush_events() 

    def close(self, event=None): 
        if self.is_open():
            self._is_window_open = False
            plt.close(self._fig) 
            plt.close('all')    

    def is_open(self):
        return self._is_window_open

# Serial Data Acquisition
ser = None 
rx_frame_channels_template = [np.zeros((NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP), dtype=np.float32) for _ in range(NUM_RX_ANTENNAS)]

def setup_serial(port, baud_rate):
    global ser
    try:
        ser = serial.Serial(port, baud_rate, timeout=1) 
        time.sleep(2) 
        ser.reset_input_buffer() 
        return True
    except serial.SerialException as e:
        return False

def parse_frame_data_from_serial(plot_is_open_flag_func): 
    global ser 
    if not ser or not ser.is_open:
        return None, -1, True 
    
    empty_line_counter = 0
    max_empty_lines = 5 

    while plot_is_open_flag_func(): 
        try:
            line_bytes = ser.readline() 
            if not line_bytes: 
                empty_line_counter += 1
                if empty_line_counter > max_empty_lines:
                    return None, -1, False 
                time.sleep(0.001) 
                continue 
            
            empty_line_counter = 0 
            line = line_bytes.decode('utf-8', errors='ignore').strip()
            
            match = re.match(r"Frame (\d+): \[(.*)\]", line)
            if match:
                frame_num_str, data_str = match.groups()
                try:
                    flat_data_1d = np.array([float(x) for x in data_str.split(', ')], dtype=np.float32)
                except ValueError:
                    continue 
                
                if len(flat_data_1d) == EXPECTED_SAMPLES_IN_FLAT_ARRAY:
                    current_rx_frames = [np.zeros_like(tpl) for tpl in rx_frame_channels_template]
                    idx = 0; reshape_ok = True
                    for c_idx in range(NUM_CHIRPS_PER_FRAME):
                        for s_idx in range(NUM_SAMPLES_PER_CHIRP):
                            for r_ant_idx in range(NUM_RX_ANTENNAS): 
                                if idx < len(flat_data_1d):
                                    current_rx_frames[r_ant_idx][c_idx, s_idx] = flat_data_1d[idx]; idx += 1
                                else: reshape_ok = False; break 
                            if not reshape_ok: break
                        if not reshape_ok: break
                    
                    if reshape_ok: return current_rx_frames, int(frame_num_str), False 
                    continue 
                continue 
            continue 
        
        except serial.SerialException as se: return None, -1, True 
        except Exception as e: traceback.print_exc(); return None, -1, True 
    
    return None, -1, True

# Main logic
if __name__ == '__main__':
    app_state = AppState() 
    threshold_gui_root = None 

    if not setup_serial(SERIAL_PORT, BAUD_RATE):
        exit()

    threshold_gui_root = setup_threshold_gui(app_state)

    radar_config = RadarConfig(NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP,
                               START_FREQUENCY_HZ, END_FREQUENCY_HZ, PROCESSING_FRAMERATE_HZ,
                               SAMPLE_RATE_HZ, CHIRP_REPETITION_TIME_S)
    distance_algo = DistanceFFT_Algo(radar_config)
    max_range_m_calculated = distance_algo.chirpsamples * distance_algo._range_bin_length \
                             if distance_algo._range_bin_length > 0 else 10.0 
    
    plotter = Draw(radar_config, max_range_m_calculated, NUM_RX_ANTENNAS, 
                   distance_algo._range_bin_length, distance_algo._velocity_axis_mps, app_state)

    frame_delay_s = 1.0 / PROCESSING_FRAMERATE_HZ if PROCESSING_FRAMERATE_HZ > 0 else 0.01 
    frame_count = 0

    try:
        while True: 
            if not plotter.is_open(): 
                break 
            
            if app_state.gui_is_active and threshold_gui_root:
                try:
                    threshold_gui_root.update_idletasks() 
                    threshold_gui_root.update()          
                except tk.TclError: 
                    if app_state.gui_is_active: 
                        app_state.gui_is_active = False
                    threshold_gui_root = None 
            elif not app_state.gui_is_active and threshold_gui_root:
                try:
                    threshold_gui_root.destroy() 
                except tk.TclError: pass 
                threshold_gui_root = None


            start_time_frame = time.time()
            antenna_frames, parsed_frame_num, should_exit = parse_frame_data_from_serial(plotter.is_open)


            if should_exit: 
                break 
            
            if antenna_frames is not None: 
                frame_count += 1
                
                all_ant_1d_profiles_orig = []
                all_ant_2d_fft_abs_orig = []
                all_ant_rd_maps_orig = []
                
                for i_ant in range(NUM_RX_ANTENNAS):
                    raw_time_data_one_antenna = antenna_frames[i_ant]
                    dist_peak_m, profile_1d, fft_abs_2d, rd_map_abs_one_ant = \
                        distance_algo.compute_distance_and_profile(raw_time_data_one_antenna)
                    
                    all_ant_1d_profiles_orig.append(profile_1d)
                    all_ant_2d_fft_abs_orig.append(fft_abs_2d)
                    all_ant_rd_maps_orig.append(rd_map_abs_one_ant)
                
                if app_state.calibration_requested:
                    app_state.baseline_1d_profiles = [p.copy() for p in all_ant_1d_profiles_orig]
                    app_state.baseline_2d_fft_abs = [f.copy() for f in all_ant_2d_fft_abs_orig]
                    app_state.baseline_rd_maps_abs = [r.copy() for r in all_ant_rd_maps_orig]
                    app_state.filtering_enabled = True
                    app_state.calibration_requested = False 

                all_ant_1d_profiles_filt = []
                all_ant_2d_fft_abs_filt = []
                all_ant_rd_maps_filt = []

                if app_state.filtering_enabled and app_state.baseline_1d_profiles: 
                    for i_ant in range(NUM_RX_ANTENNAS):
                        filt_1d = all_ant_1d_profiles_orig[i_ant] - app_state.baseline_1d_profiles[i_ant]
                        filt_1d[filt_1d < app_state.threshold_1d_profile] = FILTERING_FLOOR_1D_PROFILE_LINEAR
                        all_ant_1d_profiles_filt.append(filt_1d)

                        filt_2d_rc = all_ant_2d_fft_abs_orig[i_ant] - app_state.baseline_2d_fft_abs[i_ant]
                        filt_2d_rc[filt_2d_rc < app_state.threshold_2d_mag] = FILTERING_FLOOR_2D_MAG_LINEAR
                        all_ant_2d_fft_abs_filt.append(filt_2d_rc)

                        filt_rd = all_ant_rd_maps_orig[i_ant] - app_state.baseline_rd_maps_abs[i_ant] 
                        filt_rd[filt_rd < app_state.threshold_2d_mag] = FILTERING_FLOOR_2D_MAG_LINEAR
                        all_ant_rd_maps_filt.append(filt_rd)
                else: 
                    for i_ant in range(NUM_RX_ANTENNAS):
                        all_ant_1d_profiles_filt.append(np.zeros_like(all_ant_1d_profiles_orig[i_ant]))
                        all_ant_2d_fft_abs_filt.append(np.full_like(all_ant_2d_fft_abs_orig[i_ant], FILTERING_FLOOR_2D_MAG_LINEAR))
                        all_ant_rd_maps_filt.append(np.full_like(all_ant_rd_maps_orig[i_ant], FILTERING_FLOOR_2D_MAG_LINEAR))   

                if plotter.is_open():
                    plotter.draw(all_ant_1d_profiles_orig, all_ant_2d_fft_abs_orig, all_ant_rd_maps_orig,
                                 all_ant_1d_profiles_filt, all_ant_2d_fft_abs_filt, all_ant_rd_maps_filt)
            
            elapsed_time_frame = time.time() - start_time_frame
            sleep_duration = frame_delay_s - elapsed_time_frame
            if sleep_duration > 0: time.sleep(sleep_duration)

    except KeyboardInterrupt: 
        print("\nData collection stopped by user (Ctrl+C).")
    except Exception as e:
        traceback.print_exc()
    finally:
        if plotter.is_open(): plotter.close() 
        
        if threshold_gui_root: 
            try:
                if threshold_gui_root.winfo_exists(): 
                    threshold_gui_root.destroy()
            except tk.TclError:
                pass 
            app_state.gui_is_active = False 

        if ser and ser.is_open: ser.close()
        print("Program finished.")