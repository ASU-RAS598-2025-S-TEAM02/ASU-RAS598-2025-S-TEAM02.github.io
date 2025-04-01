import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, firwin, lfilter

# Load IMU data from YAML file
def load_imu_data(yaml_file_path):
    """
    Load IMU data from a YAML file.

    Parameters:
        yaml_file_path (str): Path to the YAML file.

    Returns:
        tuple: Time array and acceleration arrays for x, y, and z axes.
    """
    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    imu_data = np.array(data['data'])
    time = imu_data[:, 0]
    accel_x = imu_data[:, 1]
    accel_y = imu_data[:, 2]
    accel_z = imu_data[:, 3]
    
    return time, accel_x, accel_y, accel_z

# Apply Kalman filter
def kalman_filter(data, Q=0.01, R=0.1):
    """
    Apply a Kalman filter to the data.

    Parameters:
        data (array-like): Input data to filter.
        Q (float): Process noise covariance.
        R (float): Measurement noise covariance.

    Returns:
        np.ndarray: Filtered data.
    """
    n = len(data)
    estimates = np.zeros(n)
    P = np.zeros(n)
    estimates[0] = data[0]
    P[0] = 1.0

    for k in range(1, n):
        # Prediction
        x_pred = estimates[k - 1]
        P_pred = P[k - 1] + Q

        # Update
        K = P_pred / (P_pred + R)
        estimates[k] = x_pred + K * (data[k] - x_pred)
        P[k] = (1 - K) * P_pred

    return estimates

# Apply FIR filter
def apply_fir_filter(data, numtaps=101, cutoff=5.0, fs=100.0):
    """
    Apply an FIR low-pass filter to the data.

    Parameters:
        data (array-like): Input data to filter.
        numtaps (int): Number of filter taps (window size).
        cutoff (float): Cutoff frequency in Hz.
        fs (float): Sampling frequency in Hz.

    Returns:
        np.ndarray: Filtered data.
    """
    nyquist = 0.5 * fs
    normalized_cutoff = cutoff / nyquist
    taps = firwin(numtaps, normalized_cutoff, window='hamming')
    filtered_data = lfilter(taps, 1.0, data)
    return filtered_data[numtaps // 2:]  # Compensate for filter delay

# Plot original and filtered data
def plot_results(time, original, filtered, title, ylabel):
    """
    Plot original and filtered signals.

    Parameters:
        time (array-like): Time array.
        original (array-like): Original signal.
        filtered (array-like): Filtered signal.
        title (str): Plot title.
        ylabel (str): Y-axis label.
    """
    time_trimmed = time[:len(filtered)]
    original_trimmed = original[:len(filtered)]

    plt.figure(figsize=(12, 6))
    plt.plot(time_trimmed, original_trimmed, 'b-', alpha=0.5, label='Original')
    plt.plot(time_trimmed, filtered, 'r-', label='Filtered')
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Main function
# Main function
def main():
    yaml_file_path = 'data/imu.yaml'

    try:
        # Load IMU data
        time, accel_x, accel_y, accel_z = load_imu_data(yaml_file_path)

        # Estimate sampling frequency
        fs = 1.0 / np.mean(np.diff(time))
        print(f"Estimated sampling frequency: {fs:.2f} Hz")

        # Define FIR filter parameters
        cutoff_freq = 5.0  # Cutoff frequency in Hz
        window_sizes = [51, 101, 151]  # Different window sizes (numtaps)

        # Apply FIR filters with different window sizes
        filtered_results = {}
        for numtaps in window_sizes:
            filtered_results[numtaps] = apply_fir_filter(accel_x, numtaps, cutoff_freq, fs)

        # Plot original signal and filtered results
        plt.figure(figsize=(12, 8))
        plt.plot(time, accel_x, 'b-', alpha=0.5, label='Original Signal')

        for numtaps, filtered_data in filtered_results.items():
            time_trimmed = time[:len(filtered_data)]
            plt.plot(time_trimmed, filtered_data, label=f'FIR Filter (Window Size: {numtaps})')

        plt.title('X-axis Acceleration with Different FIR Filter Window Sizes')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

