import numpy as  np
import pybullet as pb
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