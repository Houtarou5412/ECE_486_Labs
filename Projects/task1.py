import numpy as np
import time
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim

def main():
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

    # Initialize the simulator with fixed positions and obstacles just outside the environment
    env_size = 7
    obstacle_offset = 0.1
    robot = MobileManipulatorUnicycleSim(
        robot_id=1,
        robot_pose=[start_pos[0], start_pos[1], start_theta],
        pickup_location=[-2.0, -2.0],
        dropoff_location=[1.5, 2.5],
        obstacles_location=[[env_size / 2 + obstacle_offset, env_size / 2 + obstacle_offset],
                            [-env_size / 2 - obstacle_offset, -env_size / 2 - obstacle_offset]]  # Obstacles just outside the environment
    )

    # Enable interactive mode
    plt.ion()
    fig, ax = plt.subplots()

    # Plot the planned path and start/end points
    ax.plot(x, y, label='Planned Path')
    ax.scatter([start_pos[0], end_pos[0]], [start_pos[1], end_pos[1]], c='red', label='Start/End Points')
    ax.quiver(start_pos[0], start_pos[1], np.cos(start_theta), np.sin(start_theta), angles='xy', scale_units='xy', scale=1, color='blue', label='Start Orientation')
    ax.quiver(end_pos[0], end_pos[1], np.cos(end_theta), np.sin(end_theta), angles='xy', scale_units='xy', scale=1, color='green', label='End Orientation')
    robot_marker, = ax.plot([], [], 'bo', label='Robot Position')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.legend()
    ax.set_title('Kinematically Feasible Path Planning')
    ax.grid()
    ax.axis('equal')

    # Follow the planned path
    for i in range(len(t) - 1):
        dt = t[i + 1] - t[i]
        dx = cs_x(t[i + 1]) - cs_x(t[i])
        dy = cs_y(t[i + 1]) - cs_y(t[i])
        
        # Compute the linear and angular velocities
        v = np.sqrt(dx**2 + dy**2) / dt
        theta = np.arctan2(dy, dx)
        dtheta = theta - robot.robot_pose[2]
        
        # Normalize dtheta to be within -pi to pi
        while dtheta > np.pi:
            dtheta -= 2 * np.pi
        while dtheta < -np.pi:
            dtheta += 2 * np.pi
        
        omega = dtheta / dt

        # Send the computed velocities to the robot
        robot.set_mobile_base_speed_and_gripper_power(v, omega, 0.0)
        
        # Simulate the robot's movement
        robot.robot_pose[0] += v * np.cos(robot.robot_pose[2]) * dt
        robot.robot_pose[1] += v * np.sin(robot.robot_pose[2]) * dt
        robot.robot_pose[2] += omega * dt

        # Update robot position on the plot
        robot_marker.set_data([robot.robot_pose[0]], [robot.robot_pose[1]])
        plt.draw()
        plt.pause(0.01)
        
        # Adjust sleep for smoother visualization
        time.sleep(dt / 10)

    # Stop the robot at the goal position
    robot.set_mobile_base_speed_and_gripper_power(0.0, 0.0, 0.0)

    # Ensure the robot stops exactly at the goal
    robot.robot_pose = [end_pos[0], end_pos[1], end_theta]

    # Update final position on the plot
    robot_marker.set_data([robot.robot_pose[0]], [robot.robot_pose[1]])
    plt.draw()
    plt.pause(0.01)
    
    # Get the robot's current pose
    poses = robot.get_poses()
    print(f"Robot, pickup, dropoff, obstacles poses: {poses}")

    # Keep the plot open
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()