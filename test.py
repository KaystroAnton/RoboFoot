import cv2 as cv
from help import positionControl, simPositionControl, fromvVectorToAngel
import numpy as np
import matplotlib.pyplot as plt
control ="direct_control"
flag = "Simulation" # Real or Simulation
#flag = "Real"
targetPosition = [100, 500]
dt = 1/240
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPosX = np.zeros(sz)
logPosY = np.zeros(sz)
logAngel = np.zeros(sz)
reflogPosX = np.zeros(sz)
reflogPosY = np.zeros(sz)
reflogAngel = np.zeros(sz)
if (flag == "Simulation"):
    from classtest import Stand
    import pybullet as pb
    import time

    for i in range(10):
        for j in range(5):
            itter = 0
            logPosX = np.zeros(sz)
            logPosY = np.zeros(sz)
            logAngel = np.zeros(sz)
            reflogPosX = np.zeros(sz)
            reflogPosY = np.zeros(sz)
            reflogAngel = np.zeros(sz)
            # print(stand.detectAruco.__doc__) #use for info
            stand = Stand(coordinates_of_robots=[[0, 0, 0.1]])
            stand.setCamera()
            viewMatrix = pb.computeViewMatrix(cameraEyePosition=pb.getDebugVisualizerCamera()[11],
                                              cameraTargetPosition=[0, 0, 0.01], cameraUpVector=[1.0, 0, 0])
            # Это надо доделать
            aruco = pb.loadURDF(fileName='aruco.urdf', basePosition=[2, 2, 1], useFixedBase=True)
            pb.changeVisualShape(aruco, -1, textureUniqueId=pb.loadTexture('aruco_cube.png'))
            #
            stand.setControl([[100, 100]])
            while itter<sz:
                stand.setArucoOnRobot(stand.robots[0],aruco)
                image = stand.getCvImage(viewMatrix)
                det = stand.detectAruco(image)
                cv.circle(det[0], (targetPosition[0], targetPosition[1]), 4, (0, 0, 255), -1)
                cv.imshow('frame', det[0])
                if cv.waitKey(1) == ord('q'):
                    break
                try:
                    flag = simPositionControl([targetPosition[0], targetPosition[1]], det[1][0], det[2][0], i, j)
                    stand.setControl([flag[0]])
                    logPosX[itter] = det[1][0][0]
                    logPosY[itter] = det[1][0][1]
                    logAngel[itter] = det[2][0]
                    reflogAngel[itter] = fromvVectorToAngel([targetPosition[0] - det[1][0][0], 800 - targetPosition[1] - det[1][0][1]])
                except:
                    print("no aruco ")
                try:
                    print("robot angel ", det[2])
                    print(fromvVectorToAngel([targetPosition[0] - det[1][0][0], 800 - targetPosition[1] - det[1][0][1]]))
                except:
                    print("no aruco ")
                reflogPosX[itter] = targetPosition[0]
                reflogPosY[itter] = 800 - targetPosition[1]
                itter += 1
                if(flag[1] == True):
                    break
                pb.stepSimulation()
                time.sleep(dt)

            plt.subplot(3, 1, 1)
            plt.grid(True)
            plt.plot(logTime, logPosX, label="simPosX")
            plt.plot(logTime, reflogPosX, label="refPosX")
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.grid(True)
            plt.plot(logTime, logPosY, label="ssimPosY")
            plt.plot(logTime, reflogPosY, label="refPosY")
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.grid(True)
            plt.plot(logTime, logAngel, label="simAngel")
            plt.plot(logTime, reflogAngel, label="refAngel")
            plt.legend()
            name = str(control) + str(i)+','+str(j)+'maxTime_'+str(maxTime)+'.png'
            plt.savefig('C:\PythonProjects\RoboFoot\RoboFoot\plots/'+name)
            plt.close('all')
            pb.disconnect()


elif (flag == "Real"):
    from classtest import RealCamera
    from UDP1 import UDP
    udp = UDP()
    Cam = RealCamera()
    param = Cam.calibrateCamera()

    while True:
        # Display the resulting frame
        cv.imshow('realFrame', Cam.detectAruco()[0])
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        Cam.detectAruco()
        inf = Cam.detectAruco()
        try:
            udp.send(positionControl(targetPosition,inf[1][0],inf[2][0]))
        except:
            udp.send(";100,0/:")


