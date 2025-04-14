import pybullet as pb
import cv2 as cv
import numpy as np
import CameraSetUp as Cam
import ArucoDetectorSetUp as Detector
import help

# for more info about function print(name_of_the_class_instance.function_name.__doc__)
class Stand:
    'Creat a stand'
    def __init__(self,robot_file_name = "robot.urdf",
                 field_file_name = "field.urdf",
                 number_of_robots = 1,
                 coordinates_of_robots = [[0,0,0.25]]):
        'creates a field and a set number of robots at the entered positions'
        if len(coordinates_of_robots) != number_of_robots: #checking the entered data
            raise ValueError(str(self) + "the size of the list does not match the number of robots")
        pb.connect(pb.GUI)  # pb.GUI for graphical version
        pb.setGravity(0, 0, -9.8)
        self.robots = []
        self.field = pb.loadURDF(fileName= field_file_name,basePosition =[0,0,0])
        for i in range(number_of_robots):
            self.robots.append(pb.loadURDF(fileName= robot_file_name,basePosition= coordinates_of_robots[i]))
            # get rid of all the default damping forces
            for j in range(pb.getNumJoints(self.robots[i])):
                pb.changeDynamics(self.robots[i], j, linearDamping=0, angularDamping=0)

    def setCamera(self, params = Cam.cameraSetUp):
        'Sets the camera to the selected position'
        pb.resetDebugVisualizerCamera(
            cameraDistance=params[0],
            cameraYaw=-params[1],
            cameraPitch=-params[2],
            cameraTargetPosition=params[3]
        )  # changes the orientation and position of the camera, the parameters are specified in file CameraSetUp
        # without args, it sets the camera exactly above the center of the field

    def getPbImage(self,viewMatrix, params = Cam.imgSetUp):
        'returns an image from the pybullet camera with the specified viewMatrix'
        # computeViewMatrix: cameraEyePosition - coordinates of the camera, cameraTargetPosition - focus point, cameraUpVector - vector perpendicular to the camera direction
        return pb.getCameraImage(width=params[0],height=params[1],viewMatrix= viewMatrix,
                                  projectionMatrix=params[2], renderer=params[3]) # get frame from camera

    def getCvImage(self,viewMatrix, params = Cam.imgSetUp):
        'returns a resheped and gray image from the pybullet camera with the specified viewMatrix'
        # computeViewMatrix: cameraEyePosition - coordinates of the camera, cameraTargetPosition - focus point, cameraUpVector - vector perpendicular to the camera direction
        return cv.cvtColor(help.resize_frame(self.getPbImage(viewMatrix,params)),cv.COLOR_BGR2GRAY) # get frame from camera

    def detectAruco(self,cvImage, detector= Detector.detector):
        'Using the specified detector, it returns the id of the found markers and a set of their angles.'
        (corners, ids, rejected) = detector.detectMarkers(cvImage)
        print (corners, ids, rejected)
        return [corners, ids]

    def setWheelsSpeed(self, robot, control = [100,100],  jointIndexes = [0,1], maxWheelSpeed = 10):
        'Sets the entered speeds(control) to the selected joints of the selected robot'
        if len(control) != len(jointIndexes): #checking the entered data
            raise ValueError(str(self) + "the size of the control list does not match the number of changed joints")
        for i in range(len(control)):
            pb.setJointMotorControl2(bodyIndex=robot, jointIndex=jointIndexes[i], targetVelocity=maxWheelSpeed*control[i]/100,
                                     controlMode=pb.VELOCITY_CONTROL,force=100)

    def setControl(self, control = [[-100,-100]]):
        'Sets the entered speeds on the robots wheels'
        if len(control) != len(self.robots): #checking the entered data
            raise ValueError(str(self) + "the size of the control list does not match the number of robots")
        for iterr in range(len(self.robots)):
            self.setWheelsSpeed(self.robots[iterr],control[iterr])

class RealCamera:
    'communication with real camera'

    def __init__(self,deviceIndex = 0):
        "Starts recording the camera"
        #deviceIndex - argument can be either the device index or the name of a video file
        self.cap =cv.VideoCapture(deviceIndex)

    def calibrateCamera(self,calibratePathFolder = 'C:\PythonProjects\RoboFoot\RoboFoot\calibrateimages', numberofImages = 10, boardSize = (6,9)):  # get camera parameters
        'calibrate camera,takes a set number of images from a folder located on the specified path'
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # termination criteria

        # Creating vector to store vectors of 3D points for each checkerboard image
        objpoints = []
        # Creating vector to store vectors of 2D points for each checkerboard image
        imgpoints = []

        # Defining the world coordinates for 3D points
        objp = np.zeros((1, boardSize[0] * boardSize[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:boardSize[0], 0:boardSize[1]].T.reshape(-1, 2)  # inserting the XY grid
        images = help.loadImagesFromFolder(calibratePathFolder,numberofImages)
        for img in images:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # convert frame color from BGR to gray
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, boardSize)
            if ret:  # checking that all corners are found
                objpoints.append(objp)
                # refining pixel coordinates for given 2d points.
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           criteria)  # increasing the accuracy of the angle coordinates

                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv.drawChessboardCorners(img, boardSize, corners2, ret)
            # check the grid
            #cv.imshow('img', img)
            #cv.waitKey(0)

        cv.destroyAllWindows()
        # compute parameters
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return mtx, dist, rvecs, tvecs

    def calibrateFrame(self,frame,param):
        'removes distortion from the image'
        img = frame
        h, w = img.shape[:2]
        newCameraMtx, roi = cv.getOptimalNewCameraMatrix(param[0], param[1], (w, h), 1, (w, h))
        # undistort
        dst = cv.undistort(img, param[0], param[1], None, newCameraMtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst

    def detectAruco(self,frame = None,calibrateParam = None,detector= Detector.detector):
        'Detect aruco markers in a given image, if calibrateParam has been set, removes distortion from the image before detection.'
        if self.cap.read()[0] & (frame == None):
            frame = self.cap.read()[1]
        if calibrateParam != None:
            calibratedFrame = self.calibrateFrame(frame, calibrateParam)
        else:
            calibratedFrame= frame
        returns = []
        (corners, ids, rejected) = detector.detectMarkers(calibratedFrame)
        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
                # Extract the marker corners
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                returns.append([corners,marker_id])

                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the ArUco detection
                cv.line(calibratedFrame, top_left, top_right, (0, 255, 0), 2)
                cv.line(calibratedFrame, top_right, bottom_right, (0, 255, 0), 2)
                cv.line(calibratedFrame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv.line(calibratedFrame, bottom_left, top_left, (0, 255, 0), 2)

                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv.circle(calibratedFrame, (center_x, center_y), 4, (0, 0, 255), -1)

                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv.putText(calibratedFrame, str(marker_id),
                           (top_left[0], top_left[1] - 15),
                           cv.FONT_HERSHEY_SIMPLEX,
                           0.5, (0, 255, 0), 2)

        # corners returns as [(top_left, top_right, bottom_right, bottom_left)]
        return (calibratedFrame,returns)

















