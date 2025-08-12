import numpy as np
import matplotlib.pyplot as plt
import process_model
import measure_model

# 1. Initialization

x0 = np.array([0, 0, 0])
cov0 = np.eye(3)

p1 = np.array([0, 1, 0])
p2 = np.array([1, 0, 0])
p3 = np.array([0, 0, 1])

Q = np.eye(3)
R = np.eye(4)

u = np.array([1, 1])
dt = 1

# 2. Prediction
x_next = process_model.dynamics(x0, u, dt)
A, B, W = process_model.process_Jacobian(x0, u, dt)
cov_next = A @ cov0 @ A.T + Q

# 3. Measurement Correction
H, V = measure_model.measure_Jacobian(x0, p1, p2, p3, R)

K_next = cov_next @ H.T @ np.linalg.inv(H @ cov_next @ H.T + R)

z1 = np.array([0, 0, 1, 0])

x_next += K_next @ (z1 - measure_model.observation(x_next, p1, p2, p3))


