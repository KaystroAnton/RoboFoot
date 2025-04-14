import pybullet as pb
import math
imgSide = 200
halfFieldSize = 4.0/2
cameraHeight = 3.0 # Z coordinate of camera
cameraSetUp = [3, 90 ,89.999, [0.0, 0.0, cameraHeight]] # [cameraDistance, cameraYaw, cameraPitch,cameraTargetPosition]
camera_proj_matrix = pb.computeProjectionMatrixFOV(
                    fov=2 * math.atan(halfFieldSize / cameraHeight) * 180 / math.pi,
                    aspect=1,
                    nearVal=0.0,
                    farVal=3.5)
# fov - field of thew in degrees, aspect = 1 - the frame will be square, nearVal - near plane dist, farVal - far plane dist
imgSetUp = [imgSide,imgSide,camera_proj_matrix,pb.ER_TINY_RENDERER]
# [image width,image height,viewMatrix,projectionMatrix,renderer]