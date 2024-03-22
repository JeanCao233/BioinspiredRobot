import serial
import time
import timeit
import serial.tools.list_ports
import logging
import binascii
import numpy as np
import matplotlib.pyplot as plt
from EKF3d import EKF_SLAM


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


def create_log():
    # Configure logging
    logging.basicConfig(filename='serial.log', level=logging.INFO, format='%(asctime)s - %(message)s')
    arduino = serial.Serial(port='/dev/cu.usbmodem1101',   baudrate=9600, timeout=.1)

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


def calcD_fromLog(datafile, sampling_freq):
    sampling_frequency = 100

    dt = 1.0 / sampling_frequency

    # read file with ax, ay, az, wx, wy, wz measurements from IMU
    imu_data = np.genfromtxt(datafile, delimiter=' ')

    ax = imu_data[:, 0]
    ay = imu_data[:, 1]
    az = imu_data[:, 2]

    l = len(ax)
    vx = np.zeros(l - 1)
    vy = np.zeros(l - 1)
    vz = np.zeros(l - 1)
    for i1 in range(l - 1):
        vx[i1] = (ax[i1+1] + ax[i1])*dt/2
        vy[i1] = (ay[i1+1] + ay[i1])*dt/2
        vz[i1] = (az[i1+1] + az[i1])*dt/2
    
    dx = np.zeros(l - 2)
    dy = np.zeros(l - 2)
    dz = np.zeros(l - 2)
    x = np.zeros(l - 1)
    y = np.zeros(l - 1)
    z = np.zeros(l - 1)
    for i2 in range(l - 2):
        dx[i2] = (vx[i2+1] + vx[i2])*dt/2
        dy[i2] = (vy[i2+1] + vy[i2])*dt/2
        dz[i2] = (vz[i2+1] + vz[i2])*dt/2
        x[i2+1] = x[i2] + dx[i2]
        y[i2+1] = y[i2] + dy[i2]
        z[i2+1] = z[i2] + dz[i2]
    
    fig = plt.figure(1)
    ax1 = plt.subplot(aspect='equal', projection='3d')
    ax1.plot(x, y, z, 'b--')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    plt.show()
    return x, y, z

def runEKFtest():
    start = timeit.default_timer()
    
    m3d = np.array([[0.,  0.,  0.],
                    [0.,  20., 10.],
                    [-10., 100.,  -10.],
                    [20., 200.,  70.],
                    [0,  -200,  -20.],
                    [-20, 0,  30.],
                    [10, -70, -70.],
                    [-50, -100, -60.]]).reshape(-1)
    
    '''
    minX, maxX, minY, maxY = -120., 450., -500., 50.
    map_x = np.linspace(minX, maxX, 7)
    map_y = np.linspace(minY, maxY, 7)
    map_X, map_Y = np.meshgrid(map_x, map_y)
    map_X = map_X.reshape(-1,1)
    map_Y = map_Y.reshape(-1,1)
    m = np.hstack((map_X, map_Y)).reshape((-1))
    '''

    dt = 0.01
    T = np.arange(0, 10, dt)
    n = int(len(m3d)/3)
    W = np.zeros((6+3*n, 6+3*n))                    #xyz + rpy + 3m
    W[0:6, 0:6] = dt**2 * 1 * np.eye(6)
    V = 0.1*np.eye(4*n)
    V[n:,n:] = 0.01*np.eye(3*n)                     #distance*n, theta*n, beta*n, gamma*n

    # EKF estimation
    mu_ekf = np.zeros((6+3*n, len(T)))
    mu_ekf[0:6,0] = np.array([-20.2, -221.8, -52.1, 0.1, 0.2, -0.1])
    # mu_ekf[3:,0] = m + 0.1
    mu_ekf[6:,0] = m3d + np.random.multivariate_normal(np.zeros(3*n), 0.5*np.eye(3*n))
    init_P = 1*np.eye(6+3*n)

    # initialize EKF SLAM
    slam = EKF_SLAM(mu_ekf[:,0], init_P, dt, W, V, n)
    
    # real state
    mu = np.zeros((6+3*n, len(T)))
    mu[0:6,0] = np.array([-20, -220, -50, 0, 0, 0])
    mu[6:,0] = m3d

    y_hist = np.zeros((4*n, len(T)))

    for i, t in enumerate(T):
        if i > 0:
            # real dynamics
            u = [-50*np.cos(t*3), 50, -100*np.sin(t*3), 1*np.sin(t*3), 1*np.sin(t*3), 1*np.sin(t*3)]
            # u = [0.5, 0.5*np.sin(t*0.5), 0]
            # u = [0.5, 0.5, 0]
            mu[:,i] = slam._f(mu[:,i-1], u) + \
                np.random.multivariate_normal(np.zeros(6+3*n), W)
            
            # measurements
            y = slam._h(mu[:,i]) + np.random.multivariate_normal(np.zeros(4*n), V)

            y_hist[:,i] = (y-slam._h(slam.mu))

            # apply EKF SLAM
            mu_est, _ = slam.predict_and_correct(y, u)
            mu_ekf[:,i] = mu_est

            #print("True      X, Y, psi:", mu[0:3,i])
            #print("Estimated X, Y, psi:", mu_est[0], mu_est[1], mu_est[2])
            #print("-------------------------------------------------------")
            
            #if i == 50: break


    fig = plt.figure(1)
    ax1 = plt.subplot(aspect='equal', projection='3d')
    ax1.plot(mu[0,:], mu[1,:], mu[2,:], 'b')
    ax1.plot(mu_ekf[0,:], mu_ekf[1,:], mu_ekf[2,:], 'r--')
    mf = m3d.reshape((-1,3))
    ax1.scatter(mf[:,0], mf[:,1], mf[:,2])
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-0.5, 0.5)
    ax1.set_ylim(-0.5, 0.5)
    ax1.set_zlim(-0.5, 0.5)
    ax1.set_aspect('equal', adjustable='box')

    stop = timeit.default_timer()
    print('Time: ', stop - start)

    plt.show()

if __name__ == '__main__':

    """
    Read from serial
    """
    create_log()

    """
    Calculate step displacement
    """
    x, y, z = calcD_fromLog('sensor_data.log', sampling_freq=10)

    """
    Run EKF test on randomly generated test
    """
    #runEKFtest()