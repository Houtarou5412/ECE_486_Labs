import numpy
import pandas
import matplotlib.pyplot as plot

# Load theoretical points
theoretical_trajectory = pandas.read_csv('theoretical_trajectory.csv').values
# Load points for each run 1-10
robot_trajectories = [pandas.read_csv(f'robot_trajectory_run_{i}.csv').values for i in range(1, 11)]

# Calculate errors between the real robot and the theoretical trajectory
errors_to_theoretical = []
for run in robot_trajectories:
    error = numpy.linalg.norm(run - theoretical_trajectory, axis=1)
    errors_to_theoretical.append(error)

# Calculate errors between separate runs of the trajectory
errors_between_runs = []
for i in range(len(robot_trajectories)):
    for j in range(i+1, len(robot_trajectories)):
        error = numpy.linalg.norm(robot_trajectories[i] - robot_trajectories[j], axis=1)
        errors_between_runs.append(error)

# Plot errors between runs and theoretical
plot.figure(figsize=(14, 7))
for i, error in enumerate(errors_to_theoretical):
    plot.plot(error, label=f'Run {i+1}')
plot.title('Error to Theoretical Trajectory')
plot.xlabel('Point')
plot.ylabel('Error (mm)')
plot.legend()
plot.tight_layout()
plot.show()

# Plot errors between runs
plot.figure(figsize=(14, 7))
for i, error in enumerate(errors_between_runs):
    plot.plot(error, label=f'Run Pair {i+1}')
plot.title('Error Between Runs')
plot.xlabel('Point')
plot.ylabel('Error (mm)')
plot.legend()
plot.tight_layout()
plot.show()

# Compare with Dobot Magician's repeatability
repeatability_spec = 0.2  # Repeatability spec in mm
max_error_to_theoretical = max([max(error) for error in errors_to_theoretical])
max_error_between_runs = max([max(error) for error in errors_between_runs])

print(f"Max Error to Theoretical Trajectory: {max_error_to_theoretical:.10f} mm")
print(f"Max Error Between Runs: {max_error_between_runs:.10f} mm")
print(f"Repeatability Spec: {repeatability_spec} mm")

if max_error_to_theoretical <= repeatability_spec and max_error_between_runs <= repeatability_spec:
    print("The robot meets the repeatability specification.")
else:
    print("The robot does not meet the repeatability specification.")