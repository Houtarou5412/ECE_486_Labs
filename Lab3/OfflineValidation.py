import csv
import math

#Link lengths in mm, l1 is not used in the forward kinematics calculations 
l1 = 85
l2 = 135
l3 = 147
l4 = 28

# DH parameters: (anticlockwise is -ve, clockwise is +ve)
# Link 1:   d = 0,      theta = J1,         a = 0,      alpha = -90 degrees 
# Link 2:   d = 0,      theta = -90 + J2,   a = l2,     alpha = 0
# Link 3:   d = 0,      theta = 90 + J3,    a = l3,     alpha = 0 
# Link 4:   d = 0,      theta = -(J2 + J3), a = l4,     alpha = 90 degrees

def transformation_matrix(J1, J2, J3, ground_angle): 
    # Convert angles from degrees to radians
    J1 = math.radians(J1)
    J2 = math.radians(-90+J2)
    J3 = math.radians(90+J3)
    ground_angle = math.radians(ground_angle)

    d = l2*math.cos(J2) + l3*math.cos(ground_angle) + l4 # technically it is l4*math.cos(ground_angle - (J2+J3)) = l4*math.cos(0)
    x = math.cos(J1) * d
    y = math.sin(J1) * d
    z = -l2*math.sin(J2) -l3*math.sin(ground_angle) # technically there is also l4*math.sin(ground_angle - (J2+J3)) = l4*math.sin(0) = 0
    
    return x, y, z

DesignData = 'Lab 3/Lab2DesignData.csv' 

# Workspace constraints function based on joint angles
def within_workspace(angles):
    J1, J2, J3 = angles
    forearmAngle = J2 + J3
    if (J1 >= -90 and J1 <= 90 and forearmAngle >= -30 and forearmAngle <= 65):
        return 1
    else:
        return 0

def move_to_position(x, y, z):
    J1, J2, J3 = IK((x, y, z))
    return within_workspace((J1, J2, J3))

def check_trajectory(start, end):
    steps = 100
    for i in range(steps + 1):
        t = i / steps
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        z = start[2] + t * (end[2] - start[2])
        J1, J2, J3 = IK((x, y, z))
        if within_workspace((J1, J2, J3)) == 0:
            return False
    return True

# Example usage of check_trajectory function
def test_trajectory():
    test_cases = [
        ((100, 100, 100), (200, 200, 200)),  # General case within workspace
        ((-90, 0, 0), (90, 70, 65)),         # Edge case at maximum joint angles
        ((0, 0, 0), (0, 70, -30)),           # Edge case at minimum forearm angle
        ((50, 50, 50), (150, 150, 150)),     # General case within workspace
        ((0, 0, 200), (0, 0, 0)),            # Edge case at maximum height
        ((0, 0, 0), (300, 300, 0)),          # Edge case at maximum radius
    ]

    for start, end in test_cases:
        if check_trajectory(start, end):
            print(f"Trajectory from {start} to {end} is within workspace constraints.")
        else:
            print(f"Trajectory from {start} to {end} violates workspace constraints.")

def IK(coordinates): 
    x,y,z = coordinates[0], coordinates[1], coordinates[2]
    # Calculate joint angle 1
    J1 = math.atan2(y, x)

    x = x - l4
    # distance from the base to the target point in the XY plane
    r = math.sqrt(x**2 + y**2)
    # distance from the base to the target point in the XZ plane
    d = math.sqrt(r**2 + z**2)

    # Calculate joint angle 3
    cos_J3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
    sin_J3 = math.sqrt(1- math.pow(cos_J3, 2))
    J3 = math.atan2(sin_J3, cos_J3)

    # Calculate joint angle 2
    alpha = math.atan2(z, r)
    beta = math.asin((l3*sin_J3) / d)
    #beta = math.acos((d**2 + l2**2 - l3**2) / (2 * d * l2))
    J2 = alpha - beta

    # J1 = math.atan2(y, x)
    # dist_1 = x - l1 * math.cos(J1)
    # dist_2 = y - l1 * math.sin(J1)
    # radius = math.sqrt(dist_1**2 + dist_2**2 + z**2)

    # cos_pos6 = (radius**2 - l2**2 - l3**2) / (2 * l2 * l3)
    # if cos_pos6 < -1 or cos_pos6 > 1:
    #     raise ValueError("Cosine value out of bounds, check workspace limits")

    # pos6 = math.acos(cos_pos6)
    # J2 = math.acos(z / radius) - math.asin((l3 * math.cos(pos6)) / radius)
    # J3 = pos6 - J2

    return (math.degrees(J1), math.degrees(J2), math.degrees(J3)) 

with open(DesignData, 'r') as f:
    data = csv.reader(f)
    count = 0
    passed = 0
    failed = 0
    for row in data:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])
        J1 = float(row[3])
        J2 = float(row[4])
        pos_6 = float(row[5])
        J3 = pos_6 - J2

        ik_J1, ik_J2, ik_J3 =  IK ((x,y,z))

        # Check if calculated values are close enough to the given values
        if (math.isclose(ik_J1, J1, rel_tol=1e-3) and 
            math.isclose(ik_J2 + ik_J3, pos_6, rel_tol=1)):
            passed += 1
        else:
            failed += 1
            print(f"Failed J1, J2, J3, pos(6): \t {J1}, {J2}, {J3}, {pos_6}")
            print (f"Calculated J1, J2, J3, pos(6): \t {ik_J1}, {ik_J2}, {ik_J3}, {ik_J2 + ik_J3}")
        
        count += 1

    print(f"-----------------------------------")
    print(f"Total: {count}, Passed: {passed}, Failed: {failed}")
    print(f"-----------------------------------")

    test_trajectory()