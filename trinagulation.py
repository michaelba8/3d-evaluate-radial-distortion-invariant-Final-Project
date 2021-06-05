import numpy as np
import radial_distortion_invariant as rdi
import matplotlib.pyplot as plt
import solve_projection as sv
import trimesh
import pyrender
import cv2
import os
from numpy.linalg import inv
import point_estimation as pe
import test1
import math


def main():
    proj_mat = test1.get_projection_matrix()
    cam1, cam2, cam3, cam4, cam5, cam6 = test1.get_cameras()
    cams = [cam1, cam2, cam3, cam4, cam5, cam6]
    cams4x4 = []
    for cam in cams:
        temp = np.vstack((cam, [0, 0, 0, 1]))
        cams4x4.append(temp)
    # cams4x4=np.array(cams4x4)
    ideal_p, dist_p = top_left_pixels()
    w = 512
    h = 512
    fovy = np.pi / 3  # 45° in radians
    f = 0.5 * h / math.tan(fovy / 2)
    cx = 256
    cy = 256
    K = np.array([
        [f, 0, cx],
        [0., f, cy],
        [0., 0., 1.]
    ], dtype=np.float64)
    t = inv(cams4x4[0])[:-1, :]
    c1 = K @ t
    t = inv(cams4x4[1])[:-1, :]
    c2 = K @ t
    proj1 = proj_mat @ cams4x4[0]
    proj1 = proj1[:-1, :]
    proj2 = proj_mat @ cams4x4[1]
    proj2 = proj2[:-1, :]
    # p1=np.float32([reverse_from_pixel(ideal_p[0],512,512)]).T
    # p2=np.float32([reverse_from_pixel(ideal_p[1],512,512)]).T
    p1 = np.float32([ideal_p[0]]).T
    p2 = np.float32([ideal_p[1]]).T
    print(c1)
    print(c2)
    res = cv2.triangulatePoints(c1, c2, p1, p2)
    print(res)
    print(res / res[3])

    print()


def reverse_from_pixel(pixels, w, h):
    p = np.float32(pixels)
    if (len(pixels.shape) == 1):
        p[0] = ((2 * pixels[0]) / w) - 1
        p[1] = ((-2 * pixels[1]) / h) + 1
        return p
    for i, pix in enumerate(pixels):
        p[i, 0] = ((2 * pix[0]) / w) - 1
        p[i, 1] = ((-2 * pix[1]) / h) + 1
    return p


def top_left_pixels():
    """ideal pixesls"""
    p1 = np.array([496, 248])
    p2 = np.array([91, 338])
    p3 = np.array([482, 278])
    p4 = np.array([92, 352])
    p5 = np.array([223, 483])
    p6 = np.array([56, 273])
    ideal_pixels = [p1, p2, p3, p4, p5, p6]
    """distorted pixels"""
    p1 = np.array([408, 252])
    p2 = np.array([141, 313])
    p3 = np.array([401, 271])
    p4 = np.array([144, 322])
    p5 = np.array([234, 402])
    p6 = np.array([120, 268])
    distorted_pixels = [p1, p2, p3, p4, p5, p6]
    return np.array(ideal_pixels), np.array(distorted_pixels)


if __name__ == '__main__':
    main()
