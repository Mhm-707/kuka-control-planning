import numpy as np

d = [0.1575, 0.2025, 0.2045, 0.2155, 0.1845, 0.2155, 0.081, 0.02+.1+0.03] ## link lengths in mm
def coordinate_transform(theta):

    T_0_1 = np.matrix([[np.cos(theta[0]), -np.sin(theta[0]), 0, 0],
                       [np.sin(theta[0]), np.cos(theta[0]), 0, 0],
                       [0, 0, 1, d[0]],
                       [0, 0, 0, 1]])

    T_1_2 = np.matrix([[-np.cos(theta[1]), np.sin(theta[1]), 0, 0],
                       [0, 0, 1, 0],
                       [np.sin(theta[1]), np.cos(theta[1]), 0, d[1]],
                       [0, 0, 0, 1]])

    T_2_3 = np.matrix([[-np.cos(theta[2]), np.sin(theta[2]), 0, 0],
                       [0, 0, 1, d[2]],
                       [np.sin(theta[2]), np.cos(theta[2]), 0, 0],
                       [0, 0, 0, 1]])

    T_3_4 = np.matrix([[np.cos(theta[3]), -np.sin(theta[3]), 0, 0],
                       [0, 0, -1, 0],
                       [np.sin(theta[3]), np.cos(theta[3]), 0, d[3]],
                       [0, 0, 0, 1]])

    T_4_5 = np.matrix([[np.cos(theta[4]), -np.sin(theta[4]), 0, 0],
                       [0, 0, 1, d[4]],
                       [-np.sin(theta[4]), -np.cos(theta[4]), 0, 0],
                       [0, 0, 0, 1]])

    T_5_6 = np.matrix([[-np.cos(theta[5]), np.sin(theta[5]), 0, 0],
                       [0, 0, 1, 0],
                       [np.sin(theta[5]), np.cos(theta[5]), 0, d[5]],
                       [0, 0, 0, 1]])

    T_6_7 = np.matrix([[-np.cos(theta[6]), np.sin(theta[6]), 0, 0],
                       [0, 0, 1, d[6]],
                       [np.sin(theta[6]), np.cos(theta[6]), 0, 0],
                       [0, 0, 0, 1]])

    T_7_8 = np.matrix([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, d[7]],
                       [0, 0, 0, 1]])

    p_0 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*np.matrix([0,0,0,1]).T

    return np.squeeze(np.asarray(p_0))[:-1]
