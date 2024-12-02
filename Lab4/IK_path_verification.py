import csv
import math

#Link lengths in mm, l1 is not used in the calculations 
l1 = 85
l2 = 135
l3 = 147
l4 = 59

def IK(coordinates): 
    x,y,z = coordinates
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
    J2 = alpha - beta

    return (math.degrees(J1), math.degrees(J2), math.degrees(J3))

# Function to check if the joint angles are within the workspace limits
def IK_angles_within_workspace(q):
    J1, J2, J3 = q
    forearmAngle = J2 + J3
    if (J1 >= -90 and J1 <= 90 and forearmAngle >= -30 and forearmAngle <= 85):
        return True
    else:
        return False

# Path to the input CSV file containing the coordinates
Path_CSV = 'Lab 4/path_planning.csv'
# Path to the output CSV file to save the joint angles
Angles_CSV = 'Lab 4/joint_angles.csv'

with open(Path_CSV, 'r') as infile, open(Angles_CSV, 'w', newline='') as outfile:
    path = csv.reader(infile)
    angles = csv.writer(outfile)
    
    count = 0
    passed = 0
    failed = 0
    
    for row in path:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])

        joint_angles = IK((x, y, z))
        
        if (IK_angles_within_workspace(joint_angles)):
            passed += 1
        else:
            failed += 1
            print(f"Failed at point {count + 1} with forearm angle {joint_angles[1] + joint_angles[2]}")
        
        angles.writerow([joint_angles[0], joint_angles[1], joint_angles[2]])
        count += 1

    print(f"-----------------------------------")
    print(f"Total: {count}, Passed: {passed}, Failed: {failed}")
    print(f"-----------------------------------")