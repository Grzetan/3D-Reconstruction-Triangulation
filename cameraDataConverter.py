# This file converts stationary_camera_data.csv to position and rotation used in cameras files in triangulator.
# If you dataset has stationary_camera_data.csv but no xml file, use this script to create one.

import mathutils
import numpy as np
import cv2

data = []

with open("./dataset/R04_D2/stationary_camera_data.csv", 'r') as f:
    for line in f.readlines()[1:]:
        splitted = line.split(",")
        pos = np.zeros((3, 1))
        pos[0][0] = float(splitted[1])
        pos[1][0] = float(splitted[2])
        pos[2][0] = float(splitted[3])

        rot = np.zeros((3, 1))
        rot[0][0] = float(splitted[4])
        rot[1][0] = float(splitted[5])
        rot[2][0] = float(splitted[6])
        data.append([pos, rot])

for e in data:
    mat, _ = cv2.Rodrigues(e[1])
    mat = np.transpose(mat)
    inv_mat = mat * -1
    
    pos = np.matmul(inv_mat, e[0]).ravel()
    print(f"Pos: {pos[0]} {pos[1]} {pos[2]}")
    
    R = mathutils.Matrix()
    R[0][0] = inv_mat[0][0]
    R[0][1] = inv_mat[0][1]
    R[0][2] = inv_mat[0][2]
    R[1][0] = inv_mat[1][0]
    R[1][1] = inv_mat[1][1]
    R[1][2] = inv_mat[1][2]
    R[2][0] = inv_mat[2][0]
    R[2][1] = inv_mat[2][1]
    R[2][2] = inv_mat[2][2]

    rot_quat = R.to_quaternion()
    print("Orientation:", rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
    print()
