import numpy as np
import matplotlib.pyplot as plt

def trilateration(p1, p2, p3, r1, r2, r3):
    P1, P2, P3 = map(lambda p: np.array(p, dtype=float), (p1, p2, p3))
    
    # Unit vectors
    ex = (P2 - P1) / np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    ey = (P3 - P1 - i * ex) / np.linalg.norm(P3 - P1 - i * ex)
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)

    # Trilateration equations
    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2 - 2 * i * x) / (2 * j)

    est_pos = P1 + x * ex + y * ey
    return est_pos

def euclid_dist(p1, p2):
    p1, p2 = np.array(p1, dtype=float), np.array(p2, dtype=float)
    return np.linalg.norm(p1 - p2)

# Example points
p1 = (0, 0)
p2 = (4, 0)
p3 = (0, 3)

pr = (1.5, 1)

r1 = euclid_dist(pr, p1)
r2 = euclid_dist(pr, p2)
r3 = euclid_dist(pr, p3)


# Compute position
pos = trilateration(p1, p2, p3, r1, r2, r3)

print("Estimated position:", pos)

# Plot
fig, ax = plt.subplots()
ax.set_aspect('equal')

# Plot beacons
beacons = np.array([p1, p2, p3])
ax.scatter(beacons[:,0], beacons[:,1], c='red', label='Beacons', zorder=3)

# Plot circles
circle1 = plt.Circle(p1, r1, color='r', fill=False, linestyle='--')
circle2 = plt.Circle(p2, r2, color='g', fill=False, linestyle='--')
circle3 = plt.Circle(p3, r3, color='b', fill=False, linestyle='--')
ax.add_artist(circle1)
ax.add_artist(circle2)
ax.add_artist(circle3)

# Plot estimated position
ax.scatter(pos[0], pos[1], c='blue', marker='x', s=100, label='Estimated Position', zorder=4)

# Labels and legend
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)
ax.legend()
ax.set_title("Trilateration in 2D")

plt.show()

