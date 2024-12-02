import csv
import math
import matplotlib.pyplot as plt

def transformation_matrix(J1, J2, J3, ground_angle): 
    l2 = 135
    l3 = 147
    l4 = 59

    # Convert angles from degrees to radians
    J1 = math.radians(J1)
    J2 = math.radians(-90 + J2)
    J3 = math.radians(90 + J3)
    ground_angle = math.radians(ground_angle)

    d = l2 * math.cos(J2) + l3 * math.cos(ground_angle) + l4  # technically it is l4*math.cos(ground_angle - (J2+J3)) = l4*math.cos(0)
    x = math.cos(J1) * d
    y = math.sin(J1) * d
    z = -l2 * math.sin(J2) - l3 * math.sin(ground_angle)  # technically there is also l4*math.sin(ground_angle - (J2+J3)) = l4*math.sin(0) = 0
    
    return x, y, z

OutputData = 'Lab 2/output.csv' 

# Initialize lists to store errors and joint angles
errors = []
joint_angles = []

with open(OutputData, 'r') as f:
    data = csv.reader(f)
    count = 0
    passed = 0
    failed = 0
    tested_q = [(0,0,0), (90,0,0), (-90,0,0), (45,70,-20), (45,40,-30), (45,40,-55), 
                (45,60,5), (45, 70, -5), (45, 70, -55), (45,0,65), (45,0,-10)]
    i = 0
    for row in data:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])

        fk_x, fk_y, fk_z = transformation_matrix(tested_q[i][0], tested_q[i][1], tested_q[i][2], tested_q[i][1] + tested_q[i][2])

        # Calculate the Euclidean distance error
        total_error = math.sqrt((fk_x - x)**2 + (fk_y - y)**2 + (fk_z - z)**2)
        errors.append(total_error)
        joint_angles.append(tested_q[i])

        if math.isclose(total_error, 0, rel_tol=1):
            passed += 1
        else:
            failed += 1
            print(f"---------------------")
            print(f"Failed at index {i-1}")
            print(f"Failed Data x, y, z: \t {x}, {y}, {z}")
            print(f"Calculated x, y, z: \t {fk_x}, {fk_y}, {fk_z}")
        
        count += 1
        i += 1

    print(f"-----------------------------------")
    print(f"Total: {count}, Passed: {passed}, Failed: {failed}")
    print(f"-----------------------------------")

# Plotting the total errors with annotations
plt.figure(figsize=(12, 6))

# Plot total error
plt.plot(errors, label='Error', color='blue', marker='o')
plt.xlabel('Test Case')
plt.ylabel('Euclidean Distance Error (mm)')
plt.title('Error between theoretical and observed forward kinematics coordinates')
plt.legend()

# Annotate the plot with joint angles
for i, q in enumerate(joint_angles):
    plt.annotate(f'q={q}', (i, errors[i]), textcoords="offset points", xytext=(0,10), ha='center')

plt.tight_layout()
plt.show()
