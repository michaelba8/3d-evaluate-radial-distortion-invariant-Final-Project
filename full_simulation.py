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


def main():
    model_path = 'files/deer.obj'
    mesh = trimesh.load(model_path, force='mesh', process=False)

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

    unvalid = []
    rdi_tot_valid = 0
    rdi_err_cnt = 0
    rdi_tot_err = 0

    rdi_mv_err_cnt = 0
    rdi_mv_tot_err = 0
    rdi_mv_tot_valid = 0

    tri_err_cnt = 0
    tri_tot_err = 0

    cameras = [0, 1, 6]
    mv_cams = []
    mv_cameras_id = [0, 1, 2, 6, 7, 5]

    mv_vertices=np.zeros((0,4))
    rdi_vertices=np.zeros((0,4))
    tri_vertices=np.zeros((0,4))
    for cam_id in mv_cameras_id:
        mv_cams.append(cams[cam_id])
    mv_cams = np.array(mv_cams)
    for i in range(i_pixels[0].shape[1]):
        # ----- 3 cams RDI------
        p1 = d_pixels[cameras[0]][:, i] + 0.5
        p2 = d_pixels[cameras[1]][:, i] + 0.5
        p3 = d_pixels[cameras[2]][:, i] + 0.5
        res, ok = pe.estimate_3d_point(cams[cameras[0]], cams[cameras[1]], cams[cameras[2]], p1, p2, p3)
        print(f'RDI error :({str(i)})\n{res - np.array([vertices[i]]).T}')
        if abs(res.sum() - vertices[i].sum()) > 10:  # unvalid
            rdi_err_cnt += 1
        else:
            rdi_tot_valid += np.linalg.norm(res - np.array([vertices[i]]).T)  # add to valid errors
        # res,ok=pe.estimate_3d_point_mv(cams,pxls)
        if not ok:
            print('not ok' + str(i))
            unvalid.append(i)
        rdi_tot_err += np.linalg.norm(res - np.array([vertices[i]]).T)
        rdi_vertices=np.vstack((rdi_vertices,res.T))
        # ----- multiview RDI 6 cameras -----------
        pxls = []
        for j in mv_cameras_id:
            p = d_pixels[j][:, i] + 0.5
            pxls.append(p)
        pxls = np.array(pxls)
        mv_res, ok = pe.estimate_3d_point_mv(mv_cams, pxls)
        print(f'MV RDI error :({str(i)})\n{mv_res - np.array([vertices[i]]).T}')
        rdi_mv_tot_err+= np.linalg.norm(mv_res - np.array([vertices[i]]).T)
        if abs(mv_res.sum() - vertices[i].sum()) > 10:  # unvalid
            rdi_mv_err_cnt += 1
        else:
            rdi_mv_tot_valid += np.linalg.norm(mv_res - np.array([vertices[i]]).T)  # add to valid errors
        mv_vertices=np.vstack((mv_vertices,mv_res.T))
        # -----trinagulation----
        tr_res = tr.triangulate(cams[6], cams[7], np.array([d_pixels[6, :, i]]).T, np.array([d_pixels[7, :, i]]).T)
        tri_vertices=np.vstack((tri_vertices,tr_res.T))
        print(f'Trinagulation error({str(i)}):\n{tr_res - np.array([vertices[i]]).T}\n')
        if abs(tr_res.sum() - vertices[i].sum()) > 10:
            tri_err_cnt += 1
        tri_tot_err += np.linalg.norm(tr_res - np.array([vertices[i]]).T)

    if len(unvalid) > 0:
        print(f'Trinagulation number of estimation with error greater than 10 (unvalid estimations): {tri_err_cnt} ')

    print(f'---------------Analysis of Results:   ---------------\n ')
    print(f'total points estimated: {i_pixels[0].shape[1]}\n')

    print('----------------RDI results:   -------------------')
    print(f'RDI number of estimation with error greater than 10 (unvalid estimations): {rdi_err_cnt} ')
    print(f'RDI total error: {rdi_tot_err}')
    print(f'RDI total VALID esimations error: {rdi_tot_valid}')
    print(f'RDI avg error: {rdi_tot_err / i_pixels[0].shape[1]}')
    print(f'RDI avg valid estimation error: {rdi_tot_valid / i_pixels[0].shape[1]}\n')

    print('--------RDI Multiview Results-----------')
    print(f'Number of Multiview cameras: {len(mv_cameras_id)}')
    print(f'Multiview RDI number of estimation with error greater than 10 (unvalid estimations): {rdi_mv_err_cnt} ')
    print(f'Multiview RDI total error: {rdi_mv_tot_err}')
    print(f'Multiview RDI total VALID estimation error: {rdi_mv_tot_valid}')
    print(f'Multiview RDI avg error: {rdi_mv_tot_err / i_pixels[0].shape[1]}')
    print(f'Multiview RDI avg valid estimation error: {rdi_mv_tot_valid / i_pixels[0].shape[1]}\n')

    print('----------------Triangulation results:   -------------------')
    print(f'Triangulation number of estimation with error greater than 10 (unvalid estimations): {tri_err_cnt} ')
    print(f'Triangulation total error: {tri_tot_err}')
    print(f'Triangulation avg error: {tri_tot_err / i_pixels[0].shape[1]}')

    #return (mv_vertices,rdi_vertices,tri_vertices)
    graphic_results(mv_vertices,mesh.copy())
    graphic_results(rdi_vertices,mesh.copy())
    graphic_results(tri_vertices,mesh.copy())




def graphic_results(est_ver,mesh):
    mesh.vertices=est_ver[:,:-1]
    temp=pyrender.Mesh.from_trimesh(mesh,smooth=False)
    scene=pyrender.Scene()
    scene.add(temp)
    pyrender.Viewer(scene, use_raymond_lighting=True)



def read_pixels(pixels_folder_path):
    d_pixels = []
    i_pixels = []
    files = os.listdir(pixels_folder_path)
    for file_name in files:
        with open(os.path.join(pixels_folder_path, file_name), 'r', encoding='utf8') as file:
            data = file.read()
        pixels = []
        for line in data.splitlines():
            pixel = []
            for coord in line.split(sep=','):
                pixel.append(coord)
            pixels.append(pixel)
        if file_name.startswith('pixelsd_'):
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
