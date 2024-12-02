import csv
import math

#Link lengths in mm, l1 is not used in the forward kinematics calculations 
l2 = 135
l3 = 147
l4 = 59

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

DesignData = 'Lab 2/Lab2DesignData.csv' 

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

        fk_x, fk_y, fk_z =  transformation_matrix (J1, J2, J3, pos_6)

        # Check if calculated values are close enough to the given values
        if (math.isclose(fk_x, x, rel_tol=1e-2) and 
            math.isclose(fk_y, y, rel_tol=1e-2) and 
            math.isclose(fk_z, z, rel_tol=1e-2)):
            passed += 1
        else:
            failed += 1
            print(f"Failed Data x, y, z: \t {x}, {y}, {z}")
            print (f"Calculated x, y, z: \t {fk_x}, {fk_y}, {fk_z}")
        
        count += 1
    print(f"-----------------------------------")
    print(f"Total: {count}, Passed: {passed}, Failed: {failed}")
    print(f"-----------------------------------")