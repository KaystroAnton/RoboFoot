import numpy as  np
import cv2 as cv
import os


def loadImagesFromFolder(path_folder= 'C:\PythonProjects\RoboFoot\RoboFoot\calibrateimages', max_images=10):
    images = []
    for filename in os.listdir(path_folder):
        if len(images) >= max_images:
            break
        img_path = os.path.join(path_folder, filename)
        if os.path.isfile(img_path):  # Проверяем, является ли это файлом
            img = cv.imread(img_path)
            if img is not None:  # Проверяем, успешно ли загружено изображение
                images.append(img)
    return images















#init aruco detector
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
parameters = cv.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
detector = cv.aruco.ArucoDetector(dictionary, parameters)

def reconstruct_frame(image,length,heigth):
    new_image = [[[0,0,0] for j in range(length)] for i in range(heigth)]
    counter = 0
    for i in range(heigth):
        for j in range(length):
            for k in range(3):
                new_image[i][j][k] = image[counter]
                print(counter)
                counter+= 1
                if counter % 4 == 0:
                    counter+= 1
    return new_image

def resize_frame(frame):
    pbImg = np.resize(np.asarray(frame[2], dtype=np.uint8), (frame[0], frame[1], 4))
    cvImg = pbImg[:, :, [2, 1, 0]]
    return cvImg

def calibrate_camera(images): # get camera parameters

    board_size = (6,9) #determine the dimensions of the board (rows,columns)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001) # termination criteria

    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, board_size[0] * board_size[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) # inserting the XY grid
    prev_img_shape = None
    for img in images:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # convert frame color from BGR to gray
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, board_size)
        if ret == True: # checking that all corners are found
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria) # increasing the accuracy of the angle coordinates

            imgpoints.append(corners2)
            print(imgpoints)

            # Draw and display the corners
            img = cv.drawChessboardCorners(img, board_size, corners2, ret)

        #cv.imshow('img', img)
        #cv.waitKey(0)


    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) # compute parameters
    return mtx, dist, rvecs, tvecs



def calibrate_images(param): # remove radial and tangential distortion
    for img in load_images_from_folder():
        h, w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(param[0], param[1], (w, h), 1, (w, h))
        # undistort
        dst = cv.undistort(img, param[0], param[1], None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        #cv.imshow('calibresult.png', dst)
        #cv.waitKey(0)

def calibrate_image(param,image): # remove radial and tangential distortion
        img = image
        h, w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(param[0], param[1], (w, h), 1, (w, h))
        # undistort
        dst = cv.undistort(img, param[0], param[1], None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        #cv.imshow('calibresult.png', dst)
        #cv.waitKey(0)
        return dst

def detect_auro(param):
    # Start the video stream
    cap = cv.VideoCapture(0)
    while (True):
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, video_frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        frame = calibrate_image(param,video_frame)
        (corners, ids, rejected) = detector.detectMarkers(frame)
        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
                # Extract the marker corners
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the ArUco detection
                cv.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv.putText(frame, str(marker_id),
                            (top_left[0], top_left[1] - 15),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            # Display the resulting frame
        cv.imshow('frame', frame)

        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        # Close down the video stream
    cap.release()
    cv.destroyAllWindows()






