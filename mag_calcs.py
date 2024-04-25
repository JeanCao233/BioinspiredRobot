import numpy as np

def calc_mag_disp(mag_data):
    x, y, z = mag_data
    dist = np.sqrt((x)**2 + (y)**2 + (z)**2)
    print("x=", x, "y=", y, "z=", z, "dist=", dist)
    #dist = np.log(dist/3644.6)/(-37.66)
    alpha = 0
    beta = 0
    theta = 0
    return dist, alpha, beta, theta

def calc_real_mag_disp(dist):
    #print("x=", x, "y=", y, "z=", z, "dist=", dist)
    real_dist = (dist/32668)**(-1/2.911)
    real_dist = real_dist*0.0254

    #real_dist = np.log(dist/3644.6)/(-37.66)
    
    return real_dist