import numpy as np
import pybullet as pb
import pybullet_data
import time

# parameters of simulation
L = 0.35    # length of the corpus
d = 0.4     # diameter of wheels
alpha = 0   # the angle of the robot's course in radians (relative to the X axis)
dt = 1/240  # pybullet simulation step
g = -9.8    # Gravity force
IMG_SIDE = 2
targetPosition = [[10,-15],0,0]    # [[positon x,y], oriantation angel, velocity] the aim of the robot


physicsClient = pb.connect(pb.GUI)  # pb.GUI for graphical version
pb.resetDebugVisualizerCamera(
    cameraDistance=3,
    cameraYaw=-90,
    cameraPitch=-89.999,    # No image if set on 90
    cameraTargetPosition=[0.0, 0.0, 3.0]
)   # Set сamera directly above the field
pb.setGravity(0,0, g)
field = pb.loadURDF("field.urdf",[0,0,-0.1])
#box1 = pb.loadURDF("box.urdf", [1,1,0.25])
#box2 = pb.loadURDF("box.urdf", [4,0,0.25])
robot1 = pb.loadURDF("robot.urdf",[0,0,0])


# get rid of all the default damping forces
#pb.changeDynamics(robot1, 1, linearDamping=0, angularDamping=0)
#pb.changeDynamics(robot1, 2, linearDamping=0, angularDamping=0)

pb.setJointMotorControl2(bodyIndex=robot1, jointIndex=1, targetVelocity= 10, controlMode=pb.VELOCITY_CONTROL,force=100)
pb.setJointMotorControl2(bodyIndex=robot1, jointIndex=0, targetVelocity= 10, controlMode=pb.VELOCITY_CONTROL,force=100)
pb.getAxisAngleFromQuaternion(pb.getBasePositionAndOrientation(robot1)[1])
pb.resetBasePositionAndOrientation(robot1, posObj = [0,0,0], ornObj = pb.getQuaternionFromAxisAngle([0.0, 0.0, 1.0], 0))




while True:
    alpha =round(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(robot1)[1])[2],3)
    pb.stepSimulation()
    print(pb.getBasePositionAndOrientation(robot1))
    #print(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(3)[1]), "dfgdfg")
    #print(pb.getAxisAngleFromQuaternion(pb.getBasePositionAndOrientation(3)[1]), "dfgdfg11111")
    img = pb.getCameraImage(
        width=IMG_SIDE+10,
        height=IMG_SIDE+10,
        viewMatrix=pb.getDebugVisualizerCamera()[2],
        projectionMatrix=pb.getDebugVisualizerCamera()[3],
        renderer=pb.ER_BULLET_HARDWARE_OPENGL
    )

    print(alpha)
    print(pb.getDebugVisualizerCamera())
    time.sleep(dt)

pb.disconnect()