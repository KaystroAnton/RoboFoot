import numpy as  np
import cv2 as cv
import os

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

def get_frame(frame):
    pbImg = np.resize(np.asarray(frame[2], dtype=np.uint8), (frame[0], frame[1], 4))
    cvImg = pbImg[:, :, [2, 1, 0]]
    return cvImg

def load_images_from_folder(path_folder= 'C:\PythonProjects\RoboFoot\RoboFoot\calibrateimages', max_images=10):
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
        ret, corners = cv.findChessboardCorners(gray, board_size) # Здесь я изменил, нужно посмотреть чтобы всё было хорошо, потом удалить запись
        if ret == True: # checking that all corners are found
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria) # шncreasing the accuracy of the angle coordinates

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv.drawChessboardCorners(img, board_size, corners2, ret)

        cv.imshow('img', img)
        cv.waitKey(0)


    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) # compute parameters
    return mtx, dist, rvecs, tvecs



def calibrate_image(param): # remove radial and tangential distortion
    images = load_images_from_folder()
    for img in images:
        h, w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(param[0], param[1], (w, h), 1, (w, h))
        # undistort
        dst = cv.undistort(img, param[0], param[1], None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        cv.imshow('calibresult.png', dst)
        cv.waitKey(0)







