import os
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim

def main():
    # The initial and final positions and orientations
    start_pos = np.array([-2, -2])
    start_theta = 0
    end_pos = np.array([1.5, 2.5])
    end_theta = -np.pi / 6

    # Time vector
    t = np.linspace(0, 10, 100)

    # Boundary conditions for cubic spline interpolation
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

    # Directory to save plots
    plot_dir = 'plots/task_1_plots/'
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)

    # Plot the planned path and start/end points
    plt.figure()
    plt.plot(x, y, label='Planned Path')
    plt.scatter([start_pos[0], end_pos[0]], [start_pos[1], end_pos[1]], c='red', label='Start/End Points')
    plt.quiver(start_pos[0], start_pos[1], np.cos(start_theta), np.sin(start_theta), angles='xy', scale_units='xy', scale=1, color='blue', label='Start Orientation')
    plt.quiver(end_pos[0], end_pos[1], np.cos(end_theta), np.sin(end_theta), angles='xy', scale_units='xy', scale=1, color='green', label='End Orientation')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.title('Kinematically Feasible Path Planning')
    plt.grid()
    plt.axis('equal')
    plt.savefig(os.path.join(plot_dir, 'path_planning.png'))
    plt.close()

    # Hangs because there is no controller for the simulation

    # # Follow the planned path
    # for i in range(len(t) - 1):
    #     dt = t[i + 1] - t[i]
    #     dx = cs_x(t[i + 1]) - cs_x(t[i])
    #     dy = cs_y(t[i + 1]) - cs_y(t[i])
        
    #     # Compute the linear and angular velocities
    #     v = np.sqrt(dx**2 + dy**2) / dt
    #     theta = np.arctan2(dy, dx)
    #     dtheta = theta - robot.robot_pose[2]
        
    #     # Normalize dtheta to be within -pi to pi
    #     dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        
    #     omega = dtheta / dt

    #     # Send the computed velocities to the robot
    #     robot.set_mobile_base_speed_and_gripper_power(v, omega, 0.0)
        
    #     # Simulate the robot's movement
    #     robot.robot_pose[0] += v * np.cos(robot.robot_pose[2]) * dt
    #     robot.robot_pose[1] += v * np.sin(robot.robot_pose[2]) * dt
    #     robot.robot_pose[2] += omega * dt

    # # Stop the robot at the goal position
    # robot.set_mobile_base_speed_and_gripper_power(0.0, 0.0, 0.0)

    # # Ensure the robot stops exactly at the goal
    # robot.robot_pose = [end_pos[0], end_pos[1], end_theta]

    # # Get the robot's current pose
    # poses = robot.get_poses()
    # print(f"Robot pose: {poses[0]}")
    # print(f"Pickup location: {poses[1]}")
    # print(f"Dropoff location: {poses[2]}")
    # print(f"Obstacles locations: {poses[3]}")

    # # Plot the final robot trajectory
    # robot_positions = np.array([[start_pos[0], start_pos[1]]] + [robot.robot_pose[:2]])
    # plt.figure()
    # plt.plot(x, y, label='Planned Path')
    # plt.plot(robot_positions[:, 0], robot_positions[:, 1], 'bo-', label='Robot Position')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.legend()
    # plt.title('Kinematically Feasible Path Planning - Robot Trajectory')
    # plt.grid()
    # plt.axis('equal')
    # plt.savefig(os.path.join(plot_dir, 'task1_robot_trajectory.png'))
    # plt.close()

if __name__ == '__main__':
    main()