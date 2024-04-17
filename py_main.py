import serial
import time
import timeit
import serial.tools.list_ports
import logging
import binascii
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from EKF3d import EKF_SLAM, runEKFtest, runEKF
from mag_calcs import calc_mag_disp


ports = list(serial.tools.list_ports.comports())

'''
for p in ports:
    print("======")
    print(p)
'''

'''
def write_read(x):
    arduino.write(bytes(x,   'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return   data

while True:
    num = input("Enter a number: ")
    value   = write_read(num)
    print(value)
'''

def calc_rpy(imu_data, mag_data):
    roll = np.arctan2(-imu_data[0], np.sqrt((imu_data[1] * imu_data[1]) + (imu_data[2] * imu_data[2])))
    pitch = np.arctan2(imu_data[1], np.sqrt((imu_data[0] * imu_data[0]) + (imu_data[2] * imu_data[2])))
    #roll = np.arctan2(imu_data[1], imu_data[2])
    #pitch = np.arctan2(-imu_data[0], np.sqrt((imu_data[1] * imu_data[1]) + (imu_data[2] * imu_data[2])))

    Xh = (mag_data[0] * np.cos(pitch)) + (mag_data[1] * np.sin(roll) * np.sin(pitch)) + (mag_data[2] * np.cos(roll) * np.sin(pitch))
    Yh = (mag_data[1] * np.cos(roll)) - (mag_data[2] * np.sin(roll))

    yaw = np.arctan2(Yh, Xh)
    #yaw = np.arctan2(mag_data[1], mag_data[0])

    return np.array([roll, pitch , yaw])

def calc_accel_without_g(accel_data, rpy):
    gravity = np.array([0, 0, -9.81])

    # Apply rotation to gravity vector
    r = R.from_euler('zyx', [rpy[2], rpy[0], rpy[1]], degrees=False)
    gravity_body = r.apply(gravity)

    # Subtract gravity from accelerometer readings
    acc_without_gravity = np.zeros(3)
    acc_without_gravity[0:2] = accel_data[0:2] - gravity_body[0:2]
    acc_without_gravity[2] = accel_data[2] + gravity_body[2]
    #print(accel_data, gravity_body, rpy)
    return acc_without_gravity

def clear_data_offset(start_time, end_time):
    print("Starting calibration")
    # Configure logging
    logging.basicConfig(filename='serial.log', level=logging.INFO, format='%(asctime)s - %(message)s')
    arduino = serial.Serial(port='/dev/cu.usbmodem152557701',   baudrate=19200, timeout=.1)

    with open('initial_data_offset.log', 'w') as log_file:
        try:
            while time.time() < end_time+start_time:
                #print(time.time())
                try:
                    # Read data from serial port
                    data = arduino.readline().decode().strip()
                    #print(data)
                    values = data.split(' ')
                    if not data.strip():  # Check if the line is empty or contains only whitespace
                        #print("Empty line, skipping...")
                        continue
                    #values = data.split(',')  # Split the line into individual values
                    
                except UnicodeDecodeError:
                    # Handle decoding errors
                    #print("Decoding error, skipping line...")
                    continue

                # Write data to log file
                print(data)
                accel_data = np.array([float(values[0]), float(values[1]), float(values[2])])
                mag_data = np.array([float(values[6]), float(values[7]), float(values[8])])
                rpy = calc_rpy(accel_data, mag_data)
                accel_without_g = calc_accel_without_g(accel_data, rpy)
                log_file.write(f"{accel_without_g[0]} {accel_without_g[1]} {accel_without_g[2]} {rpy[0]} {rpy[1]} {rpy[2]} {values[6]} {values[7]} {values[8]}\n")
                log_file.flush()  # Flush buffer to ensure data is written immediately

                # Print data to console (optional)
                #print(data)
            print("Calibration done")
        except KeyboardInterrupt:
            arduino.close()


def print_cali_data(mean_vals, start_time, end_time):
    odom_meas = np.array([[0, 0, 0, 0, 0, 0]])
    # Configure logging
    logging.basicConfig(filename='serial.log', level=logging.INFO, format='%(asctime)s - %(message)s')
    arduino = serial.Serial(port='/dev/cu.usbmodem152557701',   baudrate=19200, timeout=.1)

    with open('data_log.log', 'w') as log_file:
        try:
            while time.time() < start_time+end_time:
            #while True:
                #print(time.time())
                try:
                    # Read data from serial port
                    data = arduino.readline().decode().strip()
                    #print(data)
                    values = data.split(' ')
                    if not data.strip():  # Check if the line is empty or contains only whitespace
                        #print("Empty line, skipping...")
                        continue
                    #values = data.split(',')  # Split the line into individual values
                    

                except UnicodeDecodeError:
                    # Handle decoding errors
                    print("Decoding error, skipping line...")
                    continue

                # Write data to log file
                #log_file.write(f"{values[0]} {values[1]} {values[2]} {values[3]} {values[4]} {values[5]} {values[6]} {values[7]} {values[8]}\n")
                #log_file.flush()  # Flush buffer to ensure data is written immediately

                # Print data to console (optional)
                imu_data = np.array([float(values[0]), float(values[1]), float(values[2])])
                mag_data = np.array([float(values[6]), float(values[7]), float(values[8])]) - mean_vals[6:9]

                rpy = calc_rpy(imu_data, mag_data)
                accels = calc_accel_without_g(imu_data, rpy) - mean_vals[0:3]
                dist, alpha, beta, theta = calc_mag_disp(mag_data)
                
                odom_meas_i = np.hstack((accels, mag_data))
                odom_meas_i = np.hstack((accels, np.array([dist, 0, 0])))

                odom_meas = np.append(odom_meas, np.array([odom_meas_i]), axis=0)
                log_file.write(f"{float(accels[0]):.5f} {float(accels[1]):.5f} {float(accels[2]):.5f} {float(rpy[0])-mean_vals[3]:.5f} {float(rpy[1])-mean_vals[3]:.5f} {float(rpy[2])-mean_vals[3]:.5f} {float(dist):.5f} {float(alpha):.5f} {float(beta):.5f}\n")
                log_file.flush()  # Flush buffer to ensure data is written immediately
                
                print(f"{float(accels[0]):.5f} {float(accels[1]):.5f} {float(accels[2]):.5f}\
                        {float(rpy[0])-mean_vals[3]:.5f} {float(rpy[1])-mean_vals[3]:.5f} {float(rpy[2])-mean_vals[3]:.5f}\
                        {float(dist):.5f} {float(alpha):.5f} {float(beta):.5f}\n")
        except KeyboardInterrupt:
            arduino.close()
    return odom_meas

def log_processed_data():
    # Configure logging
    logging.basicConfig(filename='serial.log', level=logging.INFO, format='%(asctime)s - %(message)s')
    arduino = serial.Serial(port='/dev/cu.usbmodem152557701',   baudrate=115200, timeout=.1)

    with open('sensor_data.log', 'w') as log_file:
        try:
            while True:
                try:
                    # Read data from serial port
                    data = arduino.readline().decode().strip()
                    print(data)
                    values = data.split(' ')
                    if not data.strip():  # Check if the line is empty or contains only whitespace
                        print("Empty line, skipping...")
                        continue
                    #values = data.split(',')  # Split the line into individual values
                    

                except UnicodeDecodeError:
                    # Handle decoding errors
                    print("Decoding error, skipping line...")
                    continue

                # Write data to log file
                log_file.write(f"{values[0]} {values[1]} {values[2]}\n")
                log_file.flush()  # Flush buffer to ensure data is written immediately

                # Print data to console (optional)
                print(data)
        except KeyboardInterrupt:
            arduino.close()

def plot_processed_data(datafile, sampling_freq=10):
    imu_data = np.genfromtxt(datafile, delimiter=' ')
    ax = imu_data[:, 0]
    ay = imu_data[:, 1]
    az = imu_data[:, 2]
    
    l = len(ax)
    dt = 1.0 / sampling_freq
    t = np.arange(0, dt*l, dt)
    print(np.shape(t))

    plt.figure(figsize=(10, 6))
    plt.title('Accelerations')
    plt.xlabel('x')
    plt.ylabel('y')

    # Plot the curves
    plt.plot(t, ax, label='ax')
    plt.plot(t, ay, label='ay')
    plt.plot(t, az, label='az')

    plt.legend()
    plt.show()
    
def calc_mean(data):
    if isinstance(data, str):
        data = np.genfromtxt(data, delimiter=' ')

    ax = data[10:, 0]
    ay = data[10:, 1]
    az = data[10:, 2]
    gx = data[10:, 3]
    gy = data[10:, 4]
    gz = data[10:, 5]

    ax_mean = np.mean(ax)
    ay_mean = np.mean(ay)
    az_mean = np.mean(az)
    gx_mean = np.mean(gx)
    gy_mean = np.mean(gy)
    gz_mean = np.mean(gz)

    if len(data[0]) > 6:
        mx = data[:, 6]
        my = data[:, 7]
        mz = data[:, 8]

        mx_mean = np.mean(mx)
        my_mean = np.mean(my)
        mz_mean = np.mean(mz)
        return np.array([ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean, mx_mean, my_mean, mz_mean])

    return np.array([ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean])


def calcD_fromLog(datafile, sampling_freq, show_fig):

    dt = 1.0 / sampling_freq

    # read file with ax, ay, az, wx, wy, wz measurements from IMU
    imu_data = np.genfromtxt(datafile, delimiter=' ')

    ax = imu_data[:, 0]
    ay = imu_data[:, 1]
    az = imu_data[:, 2]

    ax_mean, ay_mean, az_mean = calc_mean(datafile)[0:3]
    #print(ax_mean, ay_mean, az_mean)

    ax = ax - ax_mean
    ay = ay - ay_mean
    ax = az - az_mean

    l = len(ax)
    dvx = np.zeros(l - 1)
    dvy = np.zeros(l - 1)
    dvz = np.zeros(l - 1)
    v = np.zeros((l, 3))
    for i1 in range(l - 1):
        dvx[i1] = (ax[i1+1] + ax[i1])*dt/2
        dvy[i1] = (ay[i1+1] + ay[i1])*dt/2
        dvz[i1] = (az[i1+1] + az[i1])*dt/2
        v[i1+1, 0] = v[i1, 0] + dvx[i1]
        v[i1+1, 1] = v[i1, 1] + dvy[i1]
        v[i1+1, 2] = v[i1, 2] + dvz[i1]
    
    dx = np.zeros(l - 2)
    dy = np.zeros(l - 2)
    dz = np.zeros(l - 2)
    x = np.zeros(l - 1)
    y = np.zeros(l - 1)
    z = np.zeros(l - 1)
    for i2 in range(l - 2):
        dx[i2] = (v[i2+1, 0] + v[i2, 0])*dt/2
        dy[i2] = (v[i2+1, 1] + v[i2, 1])*dt/2
        dz[i2] = (v[i2+1, 2] + v[i2, 2])*dt/2
        x[i2+1] = x[i2] + dx[i2]
        y[i2+1] = y[i2] + dy[i2]
        z[i2+1] = z[i2] + dz[i2]
    #z = z*0
    if show_fig:
        fig = plt.figure(1)
        ax1 = plt.subplot(aspect='equal', projection='3d')
        ax1.plot(x, y, z, 'b--')
        ax1.scatter(x[0], y[0], z[0])
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        plt.show()
    ans = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))
    #ans = np.vstack((np.array([[0, 0, 0]]), ans))
    np.savetxt("displacements.txt", ans)
    #print(v)
    
    return ans

def calc_delta_angle_fromLog(datafile):
    data = np.genfromtxt(datafile, delimiter=' ')
    r, c = np.shape(data)
    delta_angle = np.zeros((r, 3))
    for i in range(1, r):
        delta_angle[i] = data[i, 3:6] - data[i - 1, 3:6]
    delta_angle = delta_angle[1:, :]
    #delta_angle = np.zeros_like(delta_angle)
    np.savetxt("delta_angle.txt", delta_angle)
    return delta_angle


def get_mag_dist_fromLog(datafile):
    data = np.genfromtxt(datafile, delimiter=' ')
    ans = np.hstack((data[:, 6:], np.zeros_like(data[:, 0]).reshape(-1, 1)))
    ans = ans[1:, :]
    #ans = np.ones_like(ans)
    np.savetxt("mag_dist.txt", ans)
    return ans


if __name__ == '__main__':
    start_time = time.time()
    """
    Read from serial
    """
    clear_data_offset(start_time, 5)
    mean_vals = calc_mean("initial_data_offset.log")
    print(mean_vals)
    odom_meas = print_cali_data(mean_vals, start_time+5, 10)
    means_ = calc_mean(odom_meas)
    print(means_)
    #log_processed_data()
    #plot_processed_data('sensor_data.log')
    
    

    """
    Calculate step displacement
    """
    #delta_dist = calcD_fromLog('data_log.log', sampling_freq=10, show_fig=1)
    
                #displacement_readings, magnetometer_readings = calcD_fromLog('sensor_data.log', sampling_freq=100)
    """
    Calculate step delta angle 
    """
    #delta_angle = calc_delta_angle_fromLog('data_log.log')
    #u = np.hstack((delta_dist, delta_angle))
                #print(u)
    """
    Read Magnet distance from log file
    """
    #y = get_mag_dist_fromLog('data_log.log')
         #print(np.shape(y))

    """
    Run EKF test on randomly generated test
    """
                #runEKFtest()
    #runEKF(u, y, sampling_freq=10) #currently not updating in real time