import numpy as np

def euclid_dist(p1, p2):
    p1, p2 = np.array(p1, dtype=float), np.array(p2, dtype=float)
    return np.linalg.norm(p1 - p2)

def observation(s, p1, p2, p3):
    robot_pos = s[0:1]
    robot_angle = s[2]

    d1 = euclid_dist(robot_pos, p1)
    d2 = euclid_dist(robot_pos, p2)
    d3 = euclid_dist(robot_pos, p3)

    compass_angle = robot_angle

    z = np.array([d1, d2, d3, compass_angle]).reshape((4,1))

    return z

def beacon_Jacobian(s, pbi, v):
    x = s[0]
    y = s[1]

    xbi = pbi[0]
    ybi = pbi[1]

    dhdx = (x-xbi) / np.sqrt((x-xbi)**2 + (y-ybi)**2)
    dhdy = (y-ybi) / np.sqrt((x-xbi)**2 + (y-ybi)**2)
    dhdtheta = 0

    Hbi = np.array([dhdx, dhdy, dhdtheta])

    Vbi = 1

    return Hbi, Vbi

def compass_Jacobian(s, v):
    dhdx = 0
    dhdy = 0
    dhdtheta = 1
    
    Hcomp = np.array([dhdx, dhdy, dhdtheta])
    
    Vcomp = 1

    return Hcomp, Vcomp

def measure_Jacobian(s, p1, p2, p3, v):
    Hl1, Vl1 = beacon_Jacobian(s, p1, v)
    Hl2, Vl2 = beacon_Jacobian(s, p2, v)
    Hl3, Vl3 = beacon_Jacobian(s, p3, v)

    Hcomp, Vcomp = compass_Jacobian(s, v)

    H = np.vstack([Hl1, Hl2, Hl3, Hcomp])
    V = np.diag([Vl1, Vl2, Vl3, Vcomp])

    return H, V


