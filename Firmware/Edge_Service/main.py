import time
import csv
import os
import numpy as np
import matplotliCb
matplotlib.use('Agg')  
import matplotlib.pyplot as plt
from arduino.app_utils import Bridge
from datetime import datetime

SAMPLE_RATE = 50000
NYQUIST = SAMPLE_RATE / 2

FFT_SIZE = 4096
FFT_BINS = FFT_SIZE // 2 + 1
x_axis = np.linspace(0, NYQUIST, FFT_BINS)

is_recording = False
current_csv_file = ""
trigger_file = "RECORD.txt"
last_fft_update_count = None  # detects when the MCU has a fresh FFT result ready

print("="*60)
print(" 🎛️ FILE TRIGGER DASHBOARD READY")
print(f" To START recording: Create '{trigger_file}' in the left sidebar.")
print(f" To STOP recording : Delete '{trigger_file}'.")
print(" Logging A0, A1, A2 streams to a SINGLE synced CSV file.")
print(" FFT is computed ONBOARD (MCU) - this script just polls the result.")
print("="*60 + "\n")

try:
    while True:
        # 1. CHECK DASHBOARD CONTROL
        if os.path.exists(trigger_file):
            if not is_recording:
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                current_csv_file = f"piezo_data_{timestamp_str}.csv"
                
                with open(current_csv_file, 'w', newline='') as f:
                    f.write("Timestamp,A0_Raw,A1_Raw,A2_Raw\n")
                    
                is_recording = True
                print(f"🔴 [REC] STARTED: Saving raw data to {current_csv_file}")
        else:
            if is_recording:
                is_recording = False
                print(f"⏹️ [STOP] FINISHED: Data safely saved to {current_csv_file}\n")

        # 2. PULL RAW SENSOR DATA
        payload = Bridge.call("get_new_samples_synced", 1024)
        
        if payload and len(payload) > 0:
            samples_a0 = payload[0::3]
            samples_a1 = payload[1::3]
            samples_a2 = payload[2::3]

            # Log exact timestamps and save to CSV
            if is_recording:
                now = time.time()
                sample_interval_sec = 1.0 / SAMPLE_RATE 
                batch_size = len(samples_a0)
                
                with open(current_csv_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    for i in range(batch_size):
                        exact_time = now - ((batch_size - 1 - i) * sample_interval_sec)
                        time_str = datetime.fromtimestamp(exact_time).strftime('%Y-%m-%d %H:%M:%S.%f')
                        
                        # Write synced time + all 3 analog readings
                        writer.writerow([time_str, samples_a0[i], samples_a1[i], samples_a2[i]])
        else:
            time.sleep(0.001)

        # 3. CHECK FOR A FRESH ONBOARD FFT RESULT (cheap poll, only fetch on change)
        fft_update_count = Bridge.call("get_fft_update_count")

        if fft_update_count is not None and fft_update_count != last_fft_update_count:
            last_fft_update_count = fft_update_count

            magnitude_db = Bridge.call("get_fft_result")
            if magnitude_db and len(magnitude_db) == FFT_BINS:
                magnitude_db = np.array(magnitude_db, dtype=float)

                peak_idx = np.argmax(magnitude_db)
                peak_freq = x_axis[peak_idx]

                plt.figure(figsize=(12, 6))
                plt.plot(x_axis, magnitude_db, color='#00979D')
                plt.xlim(0, NYQUIST)
                plt.ylim(-20, 80)
                plt.xlabel("Frequency (Hz)")
                plt.ylabel("Magnitude (dB)")
                plt.title(f"50kHz Piezo Spectrum (A0, onboard FFT) (Peak: {peak_freq:.2f} Hz)")
                plt.grid(True, linestyle='--', alpha=0.6)

                plt.savefig("high_speed_fft.png")
                plt.close()

except KeyboardInterrupt:
    print("\nSystem shut down.")