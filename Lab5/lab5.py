import threading
import DobotDllType as dType
from math import sin
from math import cos
import time
import csv
import math

#load dll. This time, let's name it "magician"
magician = dType.load()

#Create string array useful for connection and IO purposes
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
    
#connect to the dobot
state = dType.ConnectDobot(magician,"",115200)[0]

#print status so we can see if something goes wrong
print("Connection status: ",CON_STR[state])

#define limits - running at 100% isn't dangerous, but it's best to
#keep it slow at first
speedLimit = 40
accelLimit = 40

#define home position
xHomePos = 200
yHomePos = 0
zHomePos = 0
fourthJointHomePos = 0 #doesn't matter, we don't have a fourth joint in lab 1

xMin = xHomePos - 50
xMax = xHomePos + 50
yMin = yHomePos - 50
yMax = yHomePos + 50
zMin = zHomePos - 50
zMax = zHomePos + 50

l1 = 85
l2 = 135
l3 = 147
l4 = 27.5

# Move using coordinates
def bind_movement_to_workspace (coordinates):
    x, y, z = coordinates
    if ((x >= xMin and x <= xMax) and (y >= yMin and y <= yMax) and (z >= zMin and z <= zMax)):
        return dType.SetPTPCmd(magician, dType.PTPMode.PTPMOVLXYZMode, x, y, z, 0, isQueued = 0)[0]
    else:
        return -1

# Return joint angles required to move to given coordinates
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

# Function to check if the joint angles are within the workspace limits and execute the command
def move_using_joint_angles(joint_angles):
    J1, J2, J3 = joint_angles
    forearmAngle = J2 + J3
    if (J1 >= -90 and J1 <= 90 and forearmAngle >= -30 and forearmAngle <= 85):
        return dType.SetPTPCmd(magician, dType.PTPMode.PTPMOVJANGLEMode, J1, J2, forearmAngle, 0, isQueued=0)[0]
    else:
        return -1

if(state == dType.DobotConnect.DobotConnect_NoError):

    dType.SetQueuedCmdClear(magician); #clear the queue before sending a command
    dType.SetHOMEParams(magician,xHomePos,yHomePos,zHomePos,fourthJointHomePos,isQueued=1) #Specify the home location
    dType.SetPTPCommonParams(magician, speedLimit, accelLimit, isQueued = 1) #Set the speed and acceleration limits
    dType.SetHOMECmd(magician,0,isQueued=1) #Actually go home. We need to queue this command, or things get weird
    
    dType.SetQueuedCmdStartExec(magician) #Execute queued commands
    
    Path_CSV = './path_planning.csv'
    with open(Path_CSV, 'r') as infile:
        path = csv.reader(infile)
        
        for row in path:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            z = z - 27.5

            joint_angles = IK((x, y, z))
            execCmd = move_using_joint_angles(joint_angles)
            cmdIndx = -1

            while execCmd != cmdIndx:
                dType.dSleep(100)
                cmdIndx = dType.GetQueuedCmdCurrentIndex(magician)[0]
                # Get and print the current pose
            pose = dType.GetPose(magician)
            print(f"Robot (x, y, z) = ({pose[0]}, {pose[1]}, {pose[2]})")

    #Same as before, stop execution if anything weird was left there    
    dType.SetQueuedCmdStopExec(magician)
    
    dType.DisconnectDobot(magician)
