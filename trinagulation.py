import numpy as np
import cv2
import os
from numpy.linalg import inv
import test1
import render_vertices as rv

width=512
height=512
models_folder = 'files'
projection_folder = 'projection results'
matrices = 'cam_matrices.txt'
output_file_name = 'pixels'



def main():
    ver=np.array([[0,163.6912,73.8259,1]]).T
    model_path, proj_folder = rv.parser()
    cam_matrices_path = os.path.join(projection_folder, proj_folder, matrices)
    cams = rv.read_cam_matrices(cam_matrices_path)
    proj_mat = test1.get_projection_matrix()
    cams4x4 = []
    for cam in cams:
        temp = np.vstack((cam, [0, 0, 0, 1]))
        cams4x4.append(temp)
    K=transform_proj_to_K(proj_mat)
    cam1=transfrom_view_to_camera(cams4x4[0])
    cam2=transfrom_view_to_camera(cams4x4[1])
    print()


def triangulate_matrix(pixels1,pixels2,view1,view2,projection_matrix=None):
    cam1=transfrom_view_to_camera(view1)
    cam2=transfrom_view_to_camera(view2)
    if projection_matrix == None:
        projection_matrix=test1.get_projection_matrix()
    K=transform_proj_to_K(projection_matrix)
    pixels1=np.float32(pixels1)
    pixels2=np.float32(pixels2)
    if pixels1.shape[1] == 2:
        pixels1=pixels1.T

    if pixels2.shape[1] == 2:
        pixels2=pixels2.T

    res=triangulate(K,cam1,cam2,pixels1,pixels2)
    return res




def transfrom_view_to_camera(view):
    cam=np.zeros((3,4))
    if view.shape[0] == 3:
        view=np.vstack((view,[0,0,0,1]))
    view=inv(view)

    cam[0,0]=view[0,0]
    cam[0,1]=view[0,1]
    cam[0,2]=view[0,2]
    cam[0,3]=view[0,3]

    cam[1,0]=-view[1,0]
    cam[1,1]=-view[1,1]
    cam[1,2]=-view[1,2]
    cam[1,3]=-view[1,3]

    cam[2,0]=-view[2,0]
    cam[2,1]=-view[2,1]
    cam[2,2]=-view[2,2]
    cam[2,3]=-view[2,3]
    return cam

def transform_proj_to_K(proj):
    K=np.zeros((3,3))
    K[0,0]=proj[0,0]*width/2
    K[0,1]=0
    K[0,2]=width/2

    K[1,0]=0
    K[1,1]=proj[1,1]*height/2
    K[1,2]=height/2

    K[2,0]=0
    K[2,1]=0
    K[2,2]=1

    return K

def triangulate(view1,view2,pix1,pix2,projection_matrix=None):
    cam1=transfrom_view_to_camera(view1)
    cam2=transfrom_view_to_camera(view2)
    if projection_matrix == None:
        projection_matrix=test1.get_projection_matrix()
    K=transform_proj_to_K(projection_matrix)
    proj1=K@cam1
    proj2=K@cam2
    pix1=np.float32(pix1)
    pix2=np.float32(pix2)
    res=cv2.triangulatePoints(proj1,proj2,pix1,pix2)
    res/=res[3]
    return res




def test(K,cam,vertex):
    p=K@cam@vertex
    p/=p[2]
    return p[:2]

if __name__ == '__main__':
    main()
