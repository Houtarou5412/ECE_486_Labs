import numpy as np
import matplotlib.pyplot as plt
import csv

# Assuming z axis offset is zero for a 2D plane plotting on the base frame
workspace_xy_limits = {
    'x_min': 150,
    'x_max': 250,
    'y_min': -50,
    'y_max': 50
}

# Read points from CSV file and return points as an array
def read_points_from_csv(file_path):
    points = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            points.append((float(row[0]), float(row[1])))
    return np.array(points)

# Find connected shapes based on the distance threshold of 0.25
def find_shapes(points, threshold=0.25):
    shapes = []
    current_shape = []
    
    for i in range(len(points) - 1):
        current_shape.append(points[i])
        if np.linalg.norm(points[i] - points[i + 1]) > threshold: # new shape if above threshold
            shapes.append(np.array(current_shape))
            current_shape = []
    
    current_shape.append(points[-1]) # append the last point
    shapes.append(np.array(current_shape))
    
    return shapes

# Scale and localize the points
def scale_and_localize_shapes(shapes, workspace_xy_limits):
    all_points = np.concatenate(shapes)
    x_min, y_min = all_points.min(axis=0)
    x_max, y_max = all_points.max(axis=0)

    # scaling factors
    scale_x = (workspace_xy_limits['x_max'] - workspace_xy_limits['x_min']) / (x_max - x_min)
    scale_y = (workspace_xy_limits['y_max'] - workspace_xy_limits['y_min']) / (y_max - y_min)
    scale = min(scale_x, scale_y)

    # offsets
    offset_x = workspace_xy_limits['x_min'] - x_min * scale
    offset_y = workspace_xy_limits['y_min'] - y_min * scale

    # apply scaling and offsets
    localized_shapes = []
    for shape in shapes:
        localized_shape = shape * scale
        localized_shape[:, 0] += offset_x
        localized_shape[:, 1] += offset_y
        localized_shapes.append(localized_shape)

    return localized_shapes

# Output the scaled and localized points to a CSV file with z-coordinate of 0 indicating base frame
def write_points_to_csv(shapes, file_path):
    with open(file_path, 'w', newline='') as file:
        csv_writer = csv.writer(file)
        for shape in shapes:
            for point in shape:
                csv_writer.writerow([point[0], point[1], 0]) # Adding z coordinate as 0

def plot_shapes(shapes):
    plt.figure()
    for shape in shapes:
        plt.plot(shape[:, 0], shape[:, 1], marker='o')
    plt.xlim(workspace_xy_limits['x_min'], workspace_xy_limits['x_max'])
    plt.ylim(workspace_xy_limits['y_min'], workspace_xy_limits['y_max'])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Localized Shapes in Task Space')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def main():
    csv_file_path = 'Lab 4/lab4_data.csv'
    points = read_points_from_csv(csv_file_path)
    shapes = find_shapes(points)
    localized_shapes = scale_and_localize_shapes(shapes, workspace_xy_limits)
    
    output_csv_file_path = 'Lab 4/scaled_localized_points.csv'
    write_points_to_csv(localized_shapes, output_csv_file_path)
    
    plot_shapes(localized_shapes)

if __name__ == "__main__":
    main()
