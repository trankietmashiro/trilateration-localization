import numpy as np

def dynamics(s, u, dt):
    x = s[0]
    y = s[1]
    theta = s[2]

    v = u[0]
    omega = u[1]

    x += v*np.cos(theta)*dt
    y += v*np.sin(theta)*dt
    theta += omega*dt 

    return np.array([x, y, theta]).reshape((3,1))

def process_Jacobian(s, u, dt):
    x = s[0]
    y = s[1]
    theta = s[2]

    v = u[0]
    omega = u[1]

    A = np.array([
        [1, 0, -dt * v * np.sin(theta)],
        [0, 1,  dt * v * np.cos(theta)],
        [0, 0, 1]
    ])

    B = np.array([
        [dt * np.cos(theta), 0.0],
        [dt * np.sin(theta), 0.0],
        [0.0, dt]
    ])

    W = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    return A, B, W

