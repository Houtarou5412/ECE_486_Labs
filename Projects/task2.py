import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Define the initial and final positions and orientations
start_pos = np.array([-2, -2])
start_theta = 0
end_pos = np.array([1.5, 2.5])
end_theta = -np.pi / 6

# Define the time vector
t = np.linspace(0, 10, 100)

# Define the boundary conditions for cubic spline interpolation
# Position boundary conditions
pos_x = [start_pos[0], end_pos[0]]
pos_y = [start_pos[1], end_pos[1]]

# Derivative boundary conditions (initial and final orientations)
dx_dt = [np.cos(start_theta), np.cos(end_theta)]
dy_dt = [np.sin(start_theta), np.sin(end_theta)]

# Create cubic splines for x and y positions
cs_x = CubicSpline([0, 10], pos_x, bc_type=((1, dx_dt[0]), (1, dx_dt[1])))
cs_y = CubicSpline([0, 10], pos_y, bc_type=((1, dy_dt[0]), (1, dy_dt[1])))

# Evaluate the spline over time t
x = cs_x(t)
y = cs_y(t)

# Compute derivatives
dx_dt = cs_x.derivative()(t)
dy_dt = cs_y.derivative()(t)
d2x_dt2 = cs_x.derivative(2)(t)
d2y_dt2 = cs_y.derivative(2)(t)

# Compute input signals
v_t = np.sqrt(dx_dt**2 + dy_dt**2)
omega_t = (d2y_dt2 * dx_dt - d2x_dt2 * dy_dt) / (dx_dt**2 + dy_dt**2)

# Plot the input signals
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, v_t, label='v(t)')
plt.xlabel('Time [s]')
plt.ylabel('v(t) [m/s]')
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(t, omega_t, label='omega(t)', color='orange')
plt.xlabel('Time [s]')
plt.ylabel('omega(t) [rad/s]')
plt.legend()
plt.suptitle('Input Signals for Feedforward Controller')
plt.show()