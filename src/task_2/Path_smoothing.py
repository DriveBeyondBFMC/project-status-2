import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


waypoints_x = [0, 5, 5, 10]
waypoints_y = [0, 0, 5, 5]

cubic_spline = CubicSpline(range(len(waypoints_x)), np.array([waypoints_x, waypoints_y]).T, axis=0)

t_spline = np.linspace(0, len(waypoints_x) - 1, 200)
x_spline, y_spline = cubic_spline(t_spline).T

# Plot the cubic spline
plt.figure(figsize=(8, 6))
plt.plot(waypoints_x, waypoints_y, 'ro', label="Waypoints")
plt.plot(x_spline, y_spline, 'b-', label="Cubic Spline")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Cubic Spline for Path with One 90-Degree Turn")
plt.legend()
plt.grid()

plt.show()

