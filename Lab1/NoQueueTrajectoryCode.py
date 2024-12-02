import threading
import DobotDllType as dType
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

#Workspace
def bind_movement_to_workspace (coordinates):
    x, y, z = coordinates[0], coordinates[1], coordinates[2]
    if ((x >= xMin and x <= xMax) and (y >= yMin and y <= yMax) and (z >= zMin and z <= zMax)):
        return dType.SetPTPCmd(magician, dType.PTPMode.PTPMOVLXYZMode, x, y, z, 0, isQueued = 0)[0]
    else:
        return -1
    
def test_workspace ():
    corners = [(xMin, yMin, zMax), (xMin, yMax, zMax), (xMax, yMax, zMax), (xMax, yMin, zMax), (xMax, yMin, zMin), (xMin, yMin, zMin), (xMin, yMax, zMin), (xMax, yMax, zMin)]
    testing_points = [(xMax + 10, yMax, zMin), (xMax, yMax-10, zMin), (xMax, yMax-10, zMin + 20), (xMax+100, yMax-10, zMin + 20)]

    print(f"\nTesting the workspace\n----------------------\n")

    print(f"---Moving to each corner of workspace---")
    for corner in corners:
        execCmd = bind_movement_to_workspace(corner)
        cmdIndx = -1
        while execCmd != cmdIndx:
            dType.dSleep(100)
            cmdIndx = dType.GetQueuedCmdCurrentIndex(magician)[0]
        pose = dType.GetPose(magician)
        print(f"Robot (x, y, z) = ({pose[0]}, {pose[1]}, {pose[2]})")

    print(f"---Trying to move outside workspace for certain points, expected behaviour: coordinates should stay the same---")
    for point in testing_points:
        execCmd = bind_movement_to_workspace(point)
        cmdIndx = -1
        while execCmd != cmdIndx:
            dType.dSleep(100)
            cmdIndx = dType.GetQueuedCmdCurrentIndex(magician)[0]
            # Get and print the current pose
        pose = dType.GetPose(magician)
        print(f"Robot (x, y, z) = ({pose[0]}, {pose[1]}, {pose[2]})")
    
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

# Generate circle trajectory
radius = 30
num_points = 50
trajectory = generate_circle_trajectory(radius, xHomePos, yHomePos, zMax, num_points)

if(state == dType.DobotConnect.DobotConnect_NoError):

    dType.SetQueuedCmdClear(magician); #clear the queue before sending a command
    dType.SetHOMEParams(magician,xHomePos,yHomePos,zHomePos,fourthJointHomePos,isQueued=1) #Specify the home location
    dType.SetPTPCommonParams(magician, speedLimit, accelLimit, isQueued = 1) #Set the speed and acceleration limits
    dType.SetHOMECmd(magician,0,isQueued=1) #Actually go home. We need to queue this command, or things get weird
    
    dType.SetQueuedCmdStartExec(magician) #Execute queued commands

    #test_workspace()

    print(f"\nTrace a circle trajectory 10 times")
    print(f"-------------------------------------")

    #Move in a circle trajectory 10 times
    num_runs = 10
    for run in range(num_runs):
        print(f"\n------------------Run {run + 1}------------------")
        for point in trajectory:
            execCmd = bind_movement_to_workspace(point)
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