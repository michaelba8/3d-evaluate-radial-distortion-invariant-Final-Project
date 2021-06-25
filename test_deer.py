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


def main():
    model_path = 'files/deer.obj'
    mesh = trimesh.load(model_path, force='mesh', process=False)
    ver=np.hstack((mesh.vertices,np.ones((mesh.vertices.shape[0], 1))))
    print(ver)
    proj=get_projection_matrix()
    tmesh = pyrender.Mesh.from_trimesh(mesh, smooth=False)
    cams = get_cameras()[[0,1,2,4,5,6,8]]
    pixels=[]
    res_mat=np.zeros((0,4))
    for i,cam in enumerate(cams):
        cam_t=np.vstack((cam,[0,0,0,1]))
        pixels.append(map_to_pixel(ver,512,512,proj,cam_t)[:2])
    for i in range(pixels[0].shape[1]):
        p1=pixels[0][:,i]
        p2=pixels[1][:,i]
        p3=pixels[2][:,i]
        #pxls=[]
        #for j in range(len(pixels)):
        #    pxls.append(pixels[j][:,i])
        res,ok=pe.estimate_3d_point(cams[0],cams[1],cams[2],p1,p2,p3)
        #res,ok=pe.estimate_3d_point_mv(cams,pxls)
        if not ok:
            print('not ok'+str(i))
        res_mat=np.vstack((res_mat,res.T))
    #print(np.linalg.norm(res_mat-ver))
    diff=res_mat-ver
    diff=diff[:,:3]
    print(diff)
    return
  #  return
    # top left horn ideal pixels:
    p1 = np.array([238, 54])  #
    p2 = np.array([164, 26])  #
    p3 = np.array([169, 88])  #
    p4 = np.array([168, 369])  #
    p5 = np.array([211, 131])  #
    p6 = np.array([335, 131])  #
    p7 = np.array([198, 50])  #
    p8 = np.array([441, 31])  #
    horn_ideal = np.array([p1, p2, p3, p4, p5, p6, p7, p8]).T
    # top left horn distorted pixels:
    p1 = np.array([244, 120])  #
    p2 = np.array([198, 112])  #
    p3 = np.array([195, 140])  #
    p4 = np.array([190, 339])  #
    p5 = np.array([221, 161])  #
    p6 = np.array([314, 163])  #
    p7 = np.array([217, 120])  #
    p8 = np.array([364, 124])  #
    horn_dist = np.array([p1, p2, p3, p4, p5, p6, p7, p8]).T

    # nose ideal pixels:
    p1 = np.array([267, 173])  #
    p2 = np.array([282, 172])  #
    p3 = np.array([255, 294])  #
    p4 = np.array([256, 420])  #
    p5 = np.array([138, 242])  #
    p6 = np.array([404, 246])  #
    p7 = np.array([119, 173])  #
    p8 = np.array([393, 173])  #
    nose_ideal = np.array([p1, p2, p3, p4, p5, p6, p7, p8])
    #res, ok = pe.estimate_3d_point_mv(cams[1:5], horn_ideal[1:5])
    res,ok =pe.estimate_3d_point(cams[0],cams[1],cams[2],horn_dist[:,0],horn_dist[:,1],horn_dist[:,2])
    print(res, ok)
    print()
    return
    # nose distorted pixels:
    p1 = np.array([266, 186])  #
    p2 = np.array([278, 185])  #
    p3 = np.array([256, 291])  #
    p4 = np.array([255, 375])  #
    p5 = np.array([163, 245])  #
    p6 = np.array([366, 250])  #
    p7 = np.array([157, 197])  #
    p8 = np.array([355, 196])  #
    nose_dist = np.array([p1, p2, p3, p4, p5, p6, p7, p8])

    # tail ideal pixels:
    p1 = np.array([-1, -1])  #
    p2 = np.array([-1, -1])  #
    p3 = np.array([-1, -1])  #
    p4 = np.array([256, 122])  #
    p5 = np.array([400, 264])  #
    p6 = np.array([146, 258])  #
    p7 = np.array([247, 202])  #
    p8 = np.array([264, 202])  #
    tail_ideal = np.array([p1, p2, p3, p4, p5, p6, p7, p8])

    # tail distorted  pixels:
    p1 = np.array([-1, -1])  #
    p2 = np.array([-1, -1])  #
    p3 = np.array([-1, -1])  #
    p4 = np.array([256, 154])  #
    p5 = np.array([364, 261])  #
    p6 = np.array([168, 256])  #
    p7 = np.array([248, 207])  #
    p8 = np.array([263, 208])  #
    tail_dist = np.array([p1, p2, p3, p4, p5, p6, p7, p8])

    for i, pix1 in enumerate(horn_ideal):
        for j, pix2 in enumerate(horn_ideal):
            for k, pix3 in enumerate(horn_ideal):
                res, ok = pe.estimate_3d_point(cams[i], cams[j], cams[k], pix1, pix2, pix3)
                print(res, ok)


def map_to_pixel(point3d, w, h, projection, view):
    p = projection @ inv(view) @ point3d.T
    p = p / p[3]
    p[0] = (w / 2 * p[0] + w / 2)
    p[1] = h - (h / 2 * p[1] + h / 2)
    return p


def reverse_from_pixel(pixels, w, h):
    p = np.float32(pixels)
    if (len(pixels.shape) == 1):
        p[0] = ((2 * pixels[0]) / w) - 1
        p[1] = ((-2 * pixels[1]) / h) + 1
        return p

    for i, pix in enumerate(pixels.T):
        p[0, i] = ((2 * pix[0]) / w) - 1
        p[1, i] = ((-2 * pix[1]) / h) + 1

    return p


def get_cameras():
    """cam1"""
    Tx = 80  # verticle axis
    Ty = 100  # horizontle axis
    Tz = 300  # height axis
    Rx = np.deg2rad(0)
    Ry = np.deg2rad(-25)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam1 = rdi.calc_cam_mat_custom(arr)
    cam1t = np.vstack((cam1, [0, 0, 0, 1]))

    """cam2"""
    Tx = -80  # verticle axis
    Ty = 100  # horizontle axis
    Tz = 300  # height axis
    Rx = np.deg2rad(0)
    Ry = np.deg2rad(20)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam2 = rdi.calc_cam_mat_custom(arr)

    """cam3"""
    Tx = 0  # verticle axis
    Ty = 200  # horizontle axis
    Tz = 300  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(0)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam3 = rdi.calc_cam_mat_custom(arr)
    cam3t = np.vstack((cam3, [0, 0, 0, 1]))

    """cam4"""
    Tx = 0  # verticle axis
    Ty = 450  # horizontle axis
    Tz = 00  # height axis
    Rx = np.deg2rad(-90)
    Ry = np.deg2rad(0)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam4 = rdi.calc_cam_mat_custom(arr)

    """cam5"""
    Tx = 350  # verticle axis
    Ty = 220  # horizontle axis
    Tz = 20  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(-90)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam5 = rdi.calc_cam_mat_custom(arr)

    """cam6"""
    Tx = -350  # verticle axis
    Ty = 60  # horizontle axis
    Tz = 220  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(80)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam6 = rdi.calc_cam_mat_custom(arr)

    """cam7"""
    Tx = 95  # verticle axis
    Ty = 130  # horizontle axis
    Tz = -270  # height axis
    Rx = np.deg2rad(-10)
    Ry = np.deg2rad(210)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam7 = rdi.calc_cam_mat_custom(arr)

    """cam8"""
    Tx = -95  # verticle axis
    Ty = 130  # horizontle axis
    Tz = -270  # height axis
    Rx = np.deg2rad(-10)
    Ry = np.deg2rad(150)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam8 = rdi.calc_cam_mat_custom(arr)

    """cam9"""
    Tx = 0  # verticle axis
    Ty = 150  # horizontle axis
    Tz = 250  # height axis
    Rx = np.deg2rad(0)
    Ry = np.deg2rad(0)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam9 = rdi.calc_cam_mat_custom(arr)

    return np.array([cam1, cam2, cam3, cam4, cam5, cam6, cam7, cam8, cam9])




def get_projection_matrix():
    return np.array([[1.73205081, 0., 0., 0.],
                     [0., 1.73205081, 0., 0.],
                     [0., 0., -1., -0.1],
                     [0., 0., -1., 0.]])


if __name__ == "__main__":
    main()
