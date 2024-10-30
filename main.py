import numpy as np
import pybullet as pb
import pybullet_data
import time

dt = 1/240  # pybullet simulation step
g = -9.8    # Gravity force


physicsClient = pb.connect(pb.GUI)  # pb.GUI for graphical version
pb.setGravity(0,0,g)
field = pb.loadURDF("field.urdf",[0,0,-0.1])
box1 = pb.loadURDF("box.urdf", [1,1,0.25])
box2 = pb.loadURDF("box.urdf", [4,0,0.25])
robot1 = pb.loadURDF("robot.urdf",[2,0,1])
'''
# get rid of all the default damping forces
pb.changeDynamics(robotId, 1, linearDamping=0, angularDamping=0)
pb.changeDynamics(robotId, 2, linearDamping=0, angularDamping=0)

 # turn off the motor for the free motion
pb.setJointMotorControl2(bodyIndex=robotId, jointIndex=1, targetVelocity= 1, controlMode=pb.VELOCITY_CONTROL)

'''

while True:
    pb.stepSimulation()
    time.sleep(dt)

pb.disconnect()