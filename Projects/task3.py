import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim

# Define the state feedback controller
def state_feedback_controller(KP, KD, zd, z, zd_dot, z_dot):
    return KP * (zd - z) + KD * (zd_dot - z_dot)

# Define the initial and final positions and orientations
start_pos = np.array([-2, -2])
start_theta = 0

# Define the time vector
t = np.linspace(0, 10, 100)

# Define the boundary conditions for cubic spline interpolation
# Position boundary conditions
pos_x = [start_pos[0], 1.5]
pos_y = [start_pos[1], 2.5]

# Derivative boundary conditions (initial and final orientations)
dx_dt = [np.cos(start_theta), np.cos(-np.pi / 6)]
dy_dt = [np.sin(start_theta), np.sin(-np.pi / 6)]

# Create cubic splines for x and y positions
cs_x = CubicSpline([0, 10], pos_x, bc_type=((1, dx_dt[0]), (1, dx_dt[1])))
cs_y = CubicSpline([0, 10], pos_y, bc_type=((1, dy_dt[0]), (1, dy_dt[1])))

# Controller gains
KP = np.array([2.0, 2.0])
KD = np.array([0.3, 0.3])

# Control input limits
v_max = 0.5
omega_max = 0.5

# Initialize the simulator with fixed positions and obstacles just outside the environment
env_size = 7
obstacle_offset = 0.1
robot = MobileManipulatorUnicycleSim(
    robot_id=1,
    robot_pose=[-2.0, -2.0, 0.0],
    pickup_location=[-2.0, -2.0],
    dropoff_location=[1.5, 2.5],
    obstacles_location=[[env_size / 2 + obstacle_offset, env_size / 2 + obstacle_offset],
                        [-env_size / 2 - obstacle_offset, -env_size / 2 - obstacle_offset]]  # Obstacles just outside the environment
)

all_trajectories = []

for _ in range(100):
    
    # Draw initial conditions from a normal distribution
    w0 = np.random.normal(loc=[start_pos[0], start_pos[1], 0, 0], scale=0.1, size=(4,))
    robot.robot_pose = w0[:3]  # Initialize the robot's pose
    z = w0[:2]
    z_dot = w0[2:]

    trajectory = [z.copy()]

    for ti in t:
        zd = np.array([cs_x(ti), cs_y(ti)])
        zd_dot = np.array([cs_x.derivative()(ti), cs_y.derivative()(ti)])

        control = state_feedback_controller(KP, KD, zd, z, zd_dot, z_dot)

        # Compute the control inputs based on the feedback control
        v_t = np.clip(np.sqrt(control[0]**2 + control[1]**2), -v_max, v_max)
        omega_t = np.clip((control[1] * np.cos(robot.robot_pose[2]) - control[0] * np.sin(robot.robot_pose[2])) / (np.cos(robot.robot_pose[2])**2 + np.sin(robot.robot_pose[2])**2), -omega_max, omega_max)

        # Apply the control inputs to the robot
        robot.set_mobile_base_speed_and_gripper_power(v_t, omega_t, 0.0)

        # Simulate the robot's movement (assume a simple kinematic model for this example)
        dt = t[1] - t[0]
        robot.robot_pose[0] += v_t * np.cos(robot.robot_pose[2]) * dt
        robot.robot_pose[1] += v_t * np.sin(robot.robot_pose[2]) * dt
        robot.robot_pose[2] += omega_t * dt

        z = robot.robot_pose[:2]
        z_dot = np.array([v_t * np.cos(robot.robot_pose[2]), v_t * np.sin(robot.robot_pose[2])])

        trajectory.append(z.copy())
    all_trajectories.append(np.array(trajectory))

# Plot the trajectory
plt.figure()
for trajectory in all_trajectories:
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b')
plt.xlabel('X position')
plt.ylabel('Y position')
plt.title(f'Trajectory (KP={KP[0]}, KD={KD[0]})')
plt.show()