import numpy as np
import math

class submodule():

    def __init__(self):
        a = 1

    def face_orient(self, pc_array):

        cross_vector = np.zeros(3)

        for row in range(len(pc_array)):
            #print(len(pc_array[row]))
            if 2 < row < len(pc_array)-5:
                for column in range(len(pc_array[row])):
                    if 2 < column < len(pc_array[row]) - 5:
                        for i in [-2, -1, 0, 1, 2]:  # -5 -3 0 3  5
                            for j in [-2, -1, 0, 1, 2]:
                                center = pc_array[row + j][column + i]
                                up = pc_array[row - 1 + j][column + i]  # 4
                                right = pc_array[row + j][column + 1 + i]
                                down = pc_array[row + 1 +j][column + i]
                                left = pc_array[row + j][column - 1 + i]
                                cross_vector += self.get_norm(center, up, right, down, left)
                                #print(row, column)

        vector_length = math.dist(np.zeros(3), cross_vector)
        cross_vector /= vector_length
        tilt_angle = round(90 - math.degrees(math.acos(cross_vector[1])), 2)
        angle = round(math.degrees(math.acos(cross_vector[0])) - 90, 2)
        print('orient,', cross_vector, tilt_angle, angle)
        return angle, tilt_angle


    def get_norm(self, center, up, right, down, left):
        up1 = np.array(up) - np.array(center)
        right1 = np.array(right) - np.array(center)
        down1 = np.array(down) - np.array(center)
        left1 = np.array(left) - np.array(center)

        cross1 = np.cross(right1, up1)
        cross2 = np.cross(up1, left1)
        cross3 = np.cross(left1, down1)
        cross4 = np.cross(down1, right1)

        norm = cross1 + cross2 + cross3 + cross4

        return norm