import threading
import DobotDllType as dType
import time
import csv
import math
import numpy as np


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

#Link lengths in mm, l1 is not used in the forward kinematics calculations 
l2 = 135
l3 = 147
l4 = 59

#Workspace
def bind_movementb_to_workspace(angles):
    J1, J2, J3 = angles
    forearmAngle = J2 + J3
    if (J1 >= -90 and J1 <= 90 and J2 >= 0 and J2 <= 70 and forearmAngle >= -30 and forearmAngle <= 65):
        return dType.SetPTPCmd(magician, dType.PTPMode.PTPMOVJANGLEMode, J1, J2, forearmAngle, 0, isQueued=0)[0]
    else:
        return -1

def move_to_position(x, y, z):
    J1, J2, J3 = IK((x, y, z))
    return bind_movementb_to_workspace((J1, J2, J3))
    
def check_trajectory(start, end):
    steps = 100
    for i in range(steps + 1):
        t = i / steps
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        z = start[2] + t * (end[2] - start[2])
        J1, J2, J3 = IK((x, y, z))
        if bind_movementb_to_workspace((J1, J2, J3)) == -1:
            return False
    return True

def IK(coordinates): 
    x,y,z = coordinates[0], coordinates[1], coordinates[2]
    x = x - l4
    # distance from the base to the target point in the XY plane
    r = math.sqrt(x**2 + y**2)
    # distance from the base to the target point in the XZ plane
    d = math.sqrt(r**2 + z**2)

    # Calculate joint angle 1
    J1 = math.atan2(y, x)

    # Calculate joint angle 2
    alpha = math.atan2(z, r)
    beta = math.acos((d**2 + l2**2 - l3**2) / (2 * d * l2))
    J2 = alpha - beta

    # Calculate joint angle 3
    cos_J3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
    J3 = math.acos(cos_J3)

    return (math.degrees(J1), math.degrees(J2), math.degrees(J3)) 


if(state == dType.DobotConnect.DobotConnect_NoError):

# Control the queue: clear it, start it, stop it

    dType.SetQueuedCmdClear(magician); #clear the queue before sending a command
    
    #Specify the home location in X,Y,Z,R (R for rotation of the EE)
    #Note that this is a pretty extended position, for no reason other than to show how far out it goes
    dType.SetHOMEParams(magician,300,0,50,0,isQueued=1)
    
    
    #set the robot parameters - velocity, acceleration. Let's run it at 100% speed/acceleration this time
    dType.SetPTPCommonParams(magician, 40, 40, isQueued = 1)
    
    #Actually go home. We need to queue this command, or things get weird
    dType.SetHOMECmd(magician,0,isQueued=1)
    
    dType.SetQueuedCmdStartExec(magician)
    radis = 50
    xCenter = 250
    start_time = time.time()

    test_coordinates = [(l3, 0, l2), (30, 10, 20)]

    # test1 = [(0,0,0),(90,0,0),(-90,0,0),(45,70,-20), (45,40,-30), (45,40,-55), (45,60,5),(45, 70, -5), (45, 70, -55), (45,0,65), ((45,0,-10))]
    #Now do a run of multiple poses in XYZ space
    for element in test_coordinates:
        
            
        #cmdIndx is a flag we are using to determine if the command is done before issuing the next one
        #Since we are not queuing the commands, we instead wait for the unqueued command to change its index
        cmdIndx = -1

        joint_angles = IK(element)
        
        #this is the index of the command that will be returned once the command is DONE. Note that we are not queueing here
        execCmd = bind_movementb_to_workspace(joint_angles)

        #Wait until cmdIndx == execCmd
        while cmdIndx != execCmd:
            dType.dSleep(10) #always good to sleep
            cmdIndx = dType.GetQueuedCmdCurrentIndex(magician)[0] #update the command index
        
        #get the current pose and print it. The GetPose function returns a list. The order of what's in the list is below
        #print("(x,y,z) = (",dType.GetPose(magician)[0],",",dType.GetPose(magician)[1],",",dType.GetPose(magician)[2],",). J1 = ",dType.GetPose(magician)[3]," J2 = ",dType.GetPose(magician)[4]," J3 = ",dType.GetPose(magician)[5]," J4 = ",dType.GetPose(magician)[6],"\n"); 
        pos = dType.GetPose(magician)
        elapsed_time = (time.time()-start_time)*1000 #in milliseconds
        print(pos[0],",",pos[1],",",pos[2],",",pos[4],",",pos[5],",",pos[6],",",elapsed_time)
        
        posData = [pos[0],pos[1],pos[2],elapsed_time]
        with open('output.csv','a',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(posData)
        
        #Now one iteration is done, go back and do it again

    #Same as before, stop execution if anything weird was left there    
    dType.SetQueuedCmdStopExec(magician)
    
    dType.DisconnectDobot(magician)
