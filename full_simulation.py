import argparse
import os
import numpy as np
from numpy.linalg import inv
import cv2

import trimesh
import pyrender

import radial_distortion_invariant as rdi
import point_estimation as pe

import trinagulation as tr

models_folder = 'files'
projection_folder = 'projection results'
matrices = 'cam_matrices.txt'
output_file_name = 'pixels'
width = 512
height = 512
y_fov = np.pi / 3
pixels_folder = 'pixels'
coef = [1, 1, -1.5, 1]


def main():
    model_path = 'files/deer.obj'
    mesh = trimesh.load(model_path, force='mesh', process=False)
    ver = np.hstack((mesh.vertices, np.ones((mesh.vertices.shape[0], 1))))

    model_path, proj_folder = parser()
    cam_matrices_path = os.path.join(projection_folder, proj_folder, matrices)
    cams = read_cam_matrices(cam_matrices_path)
    """
    ----------------------------NOTE!!!!---------------------------------------
    --------get the vertices to the end of the code----------------------------
    """
    vertices_path = os.path.join(projection_folder, proj_folder, 'vertices.txt')
    vertices = read_vertices(vertices_path)
    pixels_path = os.path.join(projection_folder, proj_folder, pixels_folder)
    i_pixels, d_pixels = read_pixels(pixels_path)
    res_mat = np.zeros((0, 4))
    tri_mat = np.zeros((0, 4))
    unvalid=[]
    count = 0
    cnt = 0
    for i in range(i_pixels[0].shape[1]):
        p1 = d_pixels[3][:, i] + 0.5
        p2 = d_pixels[4][:, i] + 0.5
        p3 = d_pixels[6][:, i] + 0.5
        res, ok = pe.estimate_3d_point(cams[3], cams[4], cams[6], p1, p2, p3)
        print(f'my algorithm:({str(i)})\n{res}')
        if abs(res.sum() - vertices[i].sum()) > 10:
            print('debug')
            count += 1
        # res,ok=pe.estimate_3d_point_mv(cams,pxls)
        if not ok:
            print('not ok' + str(i))
            unvalid.append(i)
        res_mat = np.vstack((res_mat, res.T))

        # tr.triangulate(i_pixels[0],i_pixels[1],cams[0],cams[1])
        tr_res = tr.triangulate(cams[6], cams[7], np.array([d_pixels[6, :, i]]).T, np.array([d_pixels[7, :, i]]).T)
        tri_mat = np.vstack((tri_mat, tr_res.T))
        print(f'trinagulation({str(i)}):\n{tr_res}\n')
        print()
        if abs(tr_res.sum() - vertices[i].sum()) > 10:
            cnt += 1
    print(np.linalg.norm(res_mat - vertices))
    print(np.linalg.norm(tri_mat - vertices))
    print(unvalid)
    print(count)
    print(cnt)


def read_pixels(pixels_folder_path):
    d_pixels = []
    i_pixels = []
    files = os.listdir(pixels_folder_path)
    for file_name in files:
        with open(os.path.join(pixels_folder_path, file_name), 'r') as file:
            data = file.read()
        pixels = []
        for line in data.splitlines():
            pixel = []
            for coord in line.split(sep=','):
                pixel.append(coord)
            pixels.append(pixel)
        if file_name.startswith('pixelsdistorted'):
            d_pixels.append(np.array(pixels).T)
        else:
            i_pixels.append(np.array(pixels).T)
    return np.uint32(i_pixels), np.uint32(d_pixels)


def read_vertices(path):
    with open(path, 'r') as file:
        data = file.read()
    vertices = []
    for line in data.splitlines():
        ver = []
        for coord in line.split(sep=','):
            ver.append(coord)
        vertices.append(ver)
    vertices = np.float32(vertices)
    return vertices


def read_cam_matrices(matrices_file):
    with open(matrices_file, 'r') as file:
        data = file.read()
    cams = []
    for line in data.splitlines():
        args = []
        for i, arg in enumerate(line.split(sep=',')):
            if i < 3:
                args.append(float(arg))
            else:
                args.append(np.deg2rad(float(arg)))
        cam = rdi.calc_cam_mat_custom(args)
        cams.append(cam)
    return np.array(cams)


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-name', default='deer.obj', type=str)
    parser.add_argument('--res-folder', default='deer', type=str)
    args = parser.parse_args()
    if not args.model_name.endswith('.obj'):
        print('Wrong model format, the module only except obj format')
        exit(1)

    model_path = os.path.join(models_folder, args.model_name)
    '''
        TODO: add validation tests
    '''
    return model_path, args.res_folder


if __name__ == '__main__':
    main()
