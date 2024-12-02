import os
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim

def plan_path(start_pos, end_pos, start_theta, end_theta, t):
    # boundary conditions for cubic spline interpolation
    pos_x = [start_pos[0], end_pos[0]]
    pos_y = [start_pos[1], end_pos[1]]

    dx_dt = [np.cos(start_theta), np.cos(end_theta)]
    dy_dt = [np.sin(start_theta), np.sin(end_theta)]

    # create cubic splines for x and y positions
    cs_x = CubicSpline([0, t[-1]], pos_x, bc_type=((1, dx_dt[0]), (1, dx_dt[1])))
    cs_y = CubicSpline([0, t[-1]], pos_y, bc_type=((1, dy_dt[0]), (1, dy_dt[1])))

    # evaluate the spline over time t
    x = cs_x(t)
    y = cs_y(t)

    return np.column_stack((x, y))

def compute_input_signals(z_d, t):
    z_dot = np.gradient(z_d, t, axis=0)
    z_ddot = np.gradient(z_dot, t, axis=0)
    v = np.linalg.norm(z_dot, axis=1)
    omega = (z_ddot[:, 1] * z_dot[:, 0] - z_ddot[:, 0] * z_dot[:, 1]) / (z_dot[:, 0]**2 + z_dot[:, 1]**2)
    return v, omega

def feedback_control(z_d, z, z_dot_d, z_dot, Kp, Kd):
    return Kp * (z_d - z) + Kd * (z_dot_d - z_dot)

def run_simulation(t, z_d, Kp, Kd, initial_condition):
    z_dot_d = np.gradient(z_d, t, axis=0)
    z = np.zeros_like(z_d)
    z[0] = initial_condition[:2]
    z_dot = np.zeros_like(z_d)
    z_dot[0] = initial_condition[2:]
    
    for i in range(1, len(t)):
        u_w = feedback_control(z_d[i-1], z[i-1], z_dot_d[i-1], z_dot[i-1], Kp, Kd)
        dt = t[i] - t[i-1]
        z[i] = z[i-1] + u_w * dt
        z_dot[i] = np.gradient(z, t, axis=0)[i]
    
    return z

def main():
    t = np.linspace(0, 10, 100)
    start_pos = np.array([-2, -2])
    end_pos = np.array([1.5, 2.5])
    start_theta = 0
    end_theta = -np.pi / 6
    Kp = 1.0
    Kd = 0.1
    num_simulations = 100

    # Task 1: Plan path
    z_d = plan_path(start_pos, end_pos, start_theta, end_theta, t)

    # Task 2: Compute input signals
    v, omega = compute_input_signals(z_d, t)

    # Initialize the simulator
    robot = MobileManipulatorUnicycleSim(
        robot_id=1,
        robot_pose=[start_pos[0], start_pos[1], start_theta],
        pickup_location=start_pos.tolist(),
        dropoff_location=end_pos.tolist(),
        obstacles_location=[]
    )

    # Task 3: Run 100 simulations with different initial conditions
    mean_initial_condition = np.hstack((start_pos, [0, 0]))
    std_dev = 0.1
    trajectories = []

    for _ in range(num_simulations):
        initial_condition = np.random.normal(mean_initial_condition, std_dev)
        z = run_simulation(t, z_d, Kp, Kd, initial_condition)
        trajectories.append(z)

    # [lot the planned path and trajectories
    plot_dir = 'plots/task3_plots/'
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)

    plt.figure()
    plt.plot(z_d[:, 0], z_d[:, 1], label='Desired Path', linewidth=2)
    for z in trajectories:
        plt.plot(z[:, 0], z[:, 1], alpha=0.3)
    plt.scatter([start_pos[0], end_pos[0]], [start_pos[1], end_pos[1]], c='red', label='Start/End Points')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.title('Feedback Controller - 100 Simulated Trajectories')
    plt.grid()
    plt.axis('equal')
    plt.savefig(os.path.join(plot_dir, 'feedback_trajectories.png'))
    plt.close()

if __name__ == '__main__':
    main()