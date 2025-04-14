import cv2 as cv
import pybullet as pb
import time
from classtest import Stand, RealCamera

Cam = RealCamera()
param = Cam.calibrateCamera()
stand = Stand(coordinates_of_robots= [[0,0,0.25]])
stand.setCamera()
#print(stand.detectAruco.__doc__)
viewMatrix = pb.computeViewMatrix(cameraEyePosition=pb.getDebugVisualizerCamera()[11],cameraTargetPosition=[0, 0, 0.01],cameraUpVector=[1.0, 0, 0])
stand.setControl([[100,-100]])
while True:
    #cv.imshow('frame', stand.getCvFrame(viewMatrix))
    #if cv.waitKey(1) == ord('q'):
        #break
    # Display the resulting frame
    cv.imshow('realFrame', Cam.detectAruco()[0])
    #if Cam.detectAruco()[1] != []:
        #print(Cam.detectAruco()[1])
        #print(Cam.detectAruco()[1][0][0])

    #If "q" is pressed on the keyboard,
    # exit this loop
    #if cv.waitKey(1) & 0xFF == ord('q'):
       # break
    pb.stepSimulation()
    time.sleep(1/240)

