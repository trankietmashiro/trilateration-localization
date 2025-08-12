# EKF Localization with Trilateration and Compass

## Overview

This project estimates the robot's position (x, y) and heading (theta) using an Extended Kalman Filter (EKF) with:

- Distance measurements to fixed beacons (trilateration)
- Compass heading measurements (optional)

---

## State and Measurements

- **State vector:** [x, y, theta]
- **Measurement vector:** [distance1, distance2, distance3, compass_heading]

---

## Process Model

The robot moves based on control inputs (velocity and angular velocity):

- x_t = x_(t-1) + delta_t * v * cos(theta_(t-1))
- y_t = y_(t-1) + delta_t * v * sin(theta_(t-1))
- theta_t = theta_(t-1) + delta_t * omega

---

## Measurement Model

Given the state, predict expected sensor measurements:

- Distance to each beacon = sqrt((x - beacon_x)^2 + (y - beacon_y)^2)
- Heading = theta

---

## EKF Steps

1. **Predict:**

- Predict the next state using the process model.
- Predict the covariance of the state estimate.

2. **Update:**

- Compute the difference between actual and predicted measurements.
- Calculate the Kalman gain.
- Update the state estimate and covariance.

---

## Notes

- With fewer than 3 beacons, position estimates have higher uncertainty.
- Without heading measurements, heading (theta) may drift.
- EKF fuses sensor data with motion to improve estimates over time.
- Single distance measurements constrain position to a circle around the beacon.

---

## Tips

- Use at least 3 beacons for good position accuracy.
- Add a compass or gyro for heading.
- Tune noise covariances to improve filter performance.

---

If you'd like, I can help you with code examples or a Jupyter notebook!  
