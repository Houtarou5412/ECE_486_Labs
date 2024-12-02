import os
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim

def main():
    # initial and final positions and orientations
    start_pos = np.array([-2, -2])
    start_theta = 0
    end_pos = np.array([1.5, 2.5])
    end_theta = -np.pi / 6

    # time vector
    t = np.linspace(0, 10, 100)

    # boundary conditions for cubic spline interpolation
    # position boundary conditions
    pos_x = [start_pos[0], end_pos[0]]
    pos_y = [start_pos[1], end_pos[1]]

    # derivative boundary conditions (initial and final orientations)
    dx_dt = [np.cos(start_theta), np.cos(end_theta)]
    dy_dt = [np.sin(start_theta), np.sin(end_theta)]

    # create cubic splines for x and y positions
    cs_x = CubicSpline([0, 10], pos_x, bc_type=((1, dx_dt[0]), (1, dx_dt[1])))
    cs_y = CubicSpline([0, 10], pos_y, bc_type=((1, dy_dt[0]), (1, dy_dt[1])))

    # evaluate the spline over time t
    x = cs_x(t)
    y = cs_y(t)

    # derivatives
    dx_dt = cs_x.derivative()(t)
    dy_dt = cs_y.derivative()(t)
    d2x_dt2 = cs_x.derivative(2)(t)
    d2y_dt2 = cs_y.derivative(2)(t)

    # input signals
    v_t = np.sqrt(dx_dt**2 + dy_dt**2)
    omega_t = (d2y_dt2 * dx_dt - d2x_dt2 * dy_dt) / (dx_dt**2 + dy_dt**2)

    # initialize the simulator with fixed positions and obstacles just outside the environment
    robot = MobileManipulatorUnicycleSim(
        robot_id=1,
        robot_pose=[start_pos[0], start_pos[1], start_theta],
        pickup_location=[-2.0, -2.0],
        dropoff_location=[1.5, 2.5],
        obstacles_location=[]
    )

    # directory to save plots
    plot_dir = 'plots/task2_plots/'
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)

     # plot the input signals
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
    plt.savefig(os.path.join(plot_dir, 'input_signals.png'))
    plt.close()

    # Hangs because there is no controller for simulation

    # # follow the planned path
    # robot_positions = []
    # for i in range(len(t) - 1):
    #     dt = t[i + 1] - t[i]
    #     dx = cs_x(t[i + 1]) - cs_x(t[i])
    #     dy = cs_y(t[i + 1]) - cs_y(t[i])
        
    #     # linear and angular velocities
    #     v = np.sqrt(dx**2 + dy**2) / dt
    #     theta = np.arctan2(dy, dx)
    #     dtheta = theta - robot.robot_pose[2]
        
    #     # normalize dtheta to be within -pi to pi
    #     dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        
    #     omega = dtheta / dt

    #     # send the computed velocities to the robot
    #     robot.set_mobile_base_speed_and_gripper_power(v, omega, 0.0)
        
    #     # simulate the robot's movement
    #     robot.robot_pose[0] += v * np.cos(robot.robot_pose[2]) * dt
    #     robot.robot_pose[1] += v * np.sin(robot.robot_pose[2]) * dt
    #     robot.robot_pose[2] += omega * dt
    #     robot_positions.append(robot.robot_pose[:2])

    # # stop the robot at the goal position
    # robot.set_mobile_base_speed_and_gripper_power(0.0, 0.0, 0.0)

    # # ensure the robot stops exactly at the goal
    # robot.robot_pose = [end_pos[0], end_pos[1], end_theta]

    # # get the robot's current pose
    # poses = robot.get_poses()
    # print(f"Robot, pickup, dropoff, obstacles poses: {poses}")

    # # plot the final robot trajectory
    # robot_positions = np.array(robot_positions)
    # plt.plot(x, y, label='Planned Path')
    # plt.plot(robot_positions[:, 0], robot_positions[:, 1], 'bo-', label='Robot Position')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.legend()
    # plt.title('Feedforward Controller - Robot Trajectory')
    # plt.axis('equal')
    # plt.savefig(os.path.join(plot_dir, 'robot_trajectory.png'))
    # plt.close()

if __name__ == '__main__':
    main()