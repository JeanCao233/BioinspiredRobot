import numpy as np

def calc_mag_disp(mag_data, mean_vals):
    x, y, z = mag_data
    dist = np.sqrt((x)**2 + (y)**2 + (z)**2)
    #dist = np.log(dist/3644.6)/(-37.66)
    alpha = 0
    beta = 0
    theta = 0
    return dist, alpha, beta, theta
