import csv
import math

# Function to calculate circle trajectory points
def generate_circle_trajectory(radius, center_x, center_y, center_z, num_points):
    trajectory = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = center_z
        trajectory.append((x, y, z))
    return trajectory

# Parameters for the circle trajectory
radius = 30
num_points = 50
center_x = 200
center_y = 0
center_z = 50  # Use zMax from your robot setup

# Generate the trajectory
trajectory = generate_circle_trajectory(radius, center_x, center_y, center_z, num_points)

# Write the trajectory to a CSV file
csv_file = 'theoretical_trajectory.csv'
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['x', 'y', 'z'])  # Header
    for point in trajectory:
        writer.writerow(point)

print(f"Theoretical trajectory saved to {csv_file}")
