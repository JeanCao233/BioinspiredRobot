import numpy as np
from scipy import optimize
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.approx_fprime.html

class EKF_SLAM():
    def __init__(self, init_mu, init_P, dt, W, V, n):
        """Initialize EKF SLAM

        Create and initialize an EKF SLAM to estimate the robot's pose and
        the location of map features

        Args:
            init_mu: A numpy array of size (3+2*n, ). Initial guess of the mean 
            of state. 
            init_P: A numpy array of size (3+2*n, 3+2*n). Initial guess of 
            the covariance of state.
            dt: A double. The time step.
            W: A numpy array of size (3+2*n, 3+2*n). Process noise
            V: A numpy array of size (2*n, 2*n). Observation noise
            n: A int. Number of map features
            
        Returns:
            An EKF SLAM object.
        """
        self.mu = init_mu  # initial guess of state mean
        self.P = init_P  # initial guess of state covariance
        self.dt = dt  # time step
        self.W = W  # process noise 
        self.V = V  # observation noise
        self.n = n  # number of map features

    # define dynamics formula. May need to modify this to work the jacobian framework
    def x_dynamics(self, x, u):
        alpha = x[3]
        beta = x[4]
        gamma = x[5]
        x_next0 = x[0] + self.dt*(u[0]*np.cos(beta)*np.cos(gamma) + \
                                  u[1]*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)) + \
                                  u[2]*(np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma))) + \
                    self.W[0, 0]
        return x_next0
    
    def y_dynamics(self, x, u):
        alpha = x[3]
        beta = x[4]
        gamma = x[5]
        x_next1 = x[1] + self.dt*(u[0]*np.cos(beta)*np.sin(gamma) + \
                                  u[1]*(np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + \
                                  u[2]*(np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma))) + \
                    self.W[1, 1]
        return x_next1

    def z_dynamics(self, x, u):
        alpha = x[3]
        beta = x[4]
        #gamma = x[5]
        x_next2 = x[2] + self.dt*(-u[0]*np.sin(beta) + u[1]*np.sin(alpha)*np.cos(beta) + u[2]*np.cos(alpha)*np.cos(beta)) + \
                    self.W[2, 2]
        return x_next2
    
    # define dynamics formula. May need to modify this to work the jacobian framework
    def distance(self, xk, u):
        X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z = xk
        return np.sqrt((mi_x - X)**2 + (mi_y - Y)**2 + (mi_z - Z)**2 + u)
    
    def x_angle(self, xk, u):
        X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z = xk
        return self._wrap_to_pi(np.arctan2(mi_y - Y, mi_x - X) - alpha + u)
    
    def y_angle(self, xk, u):
        X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z = xk
        return self._wrap_to_pi(np.arctan2(mi_z - Z, mi_x - X) - beta + u)
    
    def z_angle(self, xk, u):
        X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z = xk
        return self._wrap_to_pi(np.arctan2(mi_z - Z, mi_y - Y) - gamma + u)
    
    
    # TODO: complete the function below
    def _f(self, x, u):
        """
        Non-linear dynamic function.

        Compute the state at next time step according to the nonlinear dynamics f.

        Args:
            x: A numpy array of size (3+2*n, ). State at current time step.
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            x_next: A numpy array of size (3+2*n, ). The state at next time step
        """
        
        x_next = np.zeros_like(x)
        x[3:6] = self._wrap_to_pi(x[3:6])

        x_next[0] = self.x_dynamics(x, u)
        x_next[1] = self.y_dynamics(x, u)
        x_next[2] = self.z_dynamics(x, u)

        x_next[3] = self._wrap_to_pi(x[3] + self.dt*u[3] + self.W[3, 3])
        x_next[4] = self._wrap_to_pi(x[4] + self.dt*u[4] + self.W[4, 4])
        x_next[4] = self._wrap_to_pi(x[5] + self.dt*u[5] + self.W[5, 5])
        x_next[6:] = x[6:]

        return x_next

    # TODO: complete the function below
    def _h(self, x):
        """
        Non-linear measurement function.

        Compute the sensor measurement according to the nonlinear function h.

        Args:
            x: A numpy array of size (3+2*n, ). State at current time step.

        Returns:
            y: A numpy array of size (2*n, ). The sensor measurement.
        """

        X = x[0, ]
        Y = x[1, ]
        Z = x[2, ]
        alpha = x[3, ]
        beta = x[4, ]
        gamma = x[5, ]
        #psi = x[2, ]
        l = self.n
        y = np.zeros(l*4)
        for i in range(l):
            mi_x = x[i*3 + 6, ]
            mi_y = x[i*3 + 7, ]
            mi_z = x[i*3 + 8, ]
            xk = np.array([X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z])
            y[i, ] = self.distance(xk, self.V[i, i])
            y[i+l, ] = self.x_angle(xk, self.V[i+l, i+l])
            y[i+2*l, ] = self.y_angle(xk, self.V[i+2*l, i+2*l])
            y[i+3*l, ] = self.z_angle(xk, self.V[i+3*l, i+3*l])
        return y

    # TODO: complete the function below
    def _compute_F(self, x, u):
        """
        Compute Jacobian of f

        Args:
            x: A numpy array of size (3+2*n, ). The state vector.
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            F: A numpy array of size (3+2*n, 3+2*n). The jacobian of f evaluated at x_k.
        """
        l = len(x)
        F = np.eye(l)
        x[3:6] = self._wrap_to_pi(x[3:6])

        F[0, :] = optimize.approx_fprime(x, self.x_dynamics, [1.5e-8]*l, u)
        F[1, :] = optimize.approx_fprime(x, self.y_dynamics, [1.5e-8]*l, u)
        F[2, :] = optimize.approx_fprime(x, self.z_dynamics, [1.5e-8]*l, u)
        
        return F

    # TODO: complete the function below
    def _compute_H(self, x):
        """
        Compute Jacobian of h

        Args:
            x: A numpy array of size (3+2*n, ). The state vector.

        Returns:
            H: A numpy array of size (2*n, 3+2*n). The jacobian of h evaluated at x_k.
        """
        x[3:6] = self._wrap_to_pi(x[3:6])

        l = self.n
        H = np.zeros((l*4, l*3 + 6))
        X = x[0, ]
        Y = x[1, ]
        Z = x[2, ]
        #psi = x[2, ]
        alpha, beta, gamma = x[3:6, ]

        for i in range(l):
            mi_x = x[i*3 + 6, ]
            mi_y = x[i*3 + 7, ]
            mi_z = x[i*3 + 8, ]
            xk = np.array([X, Y, Z, alpha, beta, gamma, mi_x, mi_y, mi_z])

            H_i = optimize.approx_fprime(xk, self.distance, [1.5e-8]*9, self.V[i, i])
            H[i, 0:6] = H_i[0:6]
            H[i, (i*3+6):(i*3+9)] = H_i[6:9]

            H_i_l = optimize.approx_fprime(xk, self.x_angle, [1.5e-8]*9, self.V[i+l, i+l])
            H[i+l, 0:6] = H_i_l[0:6]
            H[i+l, (i*3+6):(i*3+9)] = H_i_l[6:9]

            H_i_2l = optimize.approx_fprime(xk, self.y_angle, [1.5e-8]*9, self.V[i+2*l, i+2*l])
            H[i+2*l, 0:6] = H_i_2l[0:6]
            H[i+2*l, (i*3+6):(i*3+9)] = H_i_2l[6:9]

            H_i_3l = optimize.approx_fprime(xk, self.z_angle, [1.5e-8]*9, self.V[i+3*l, i+3*l])
            H[i+3*l, 0:6] = H_i_3l[0:6]
            H[i+3*l, (i*3+6):(i*3+9)] = H_i_3l[6:9]

        return H


    def predict_and_correct(self, y, u):
        """
        Predice and correct step of EKF

        Args:
            y: A numpy array of size (2*n, ). The measurements according to the project description.
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            self.mu: A numpy array of size (3+2*n, ). The corrected state estimation
            self.P: A numpy array of size (3+2*n, 3+2*n). The corrected state covariance
        """

        # compute F
        F = self._compute_F(self.mu, u)

        #***************** Predict step *****************#
        # predict the state
        self.mu = self._f(self.mu, u)
        self.mu[3:6] =  self._wrap_to_pi(self.mu[3:6])

        # predict the error covariance
        self.P = F @ self.P @ F.T + self.W

        #***************** Correct step *****************#
        # compute H matrix
        H = self._compute_H(self.mu)

        # compute the Kalman gain
        L = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.V)

        # update estimation with new measurement
        diff = y - self._h(self.mu)
        diff[self.n:] = self._wrap_to_pi(diff[self.n:])
        self.mu = self.mu + L @ diff
        self.mu[3:6] =  self._wrap_to_pi(self.mu[3:6])

        # update the error covariance
        self.P = (np.eye(6+3*self.n) - L @ H) @ self.P

        return self.mu, self.P


    def _wrap_to_pi(self, angle):
        angle = angle - 2*np.pi*np.floor((angle+np.pi )/(2*np.pi))
        return angle



if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import timeit
    import serial

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

    stop = timeit.default_timer()
    print('Time: ', stop - start)

    plt.show()

