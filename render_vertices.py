import argparse
import os
import numpy as np
from numpy.linalg import inv
import cv2

import trimesh
import pyrender

import radial_distortion_invariant as rdi
import applyDistortion as ad

# global constant variables
models_folder = 'files'
projection_folder = 'projection results'
matrices = 'cam_matrices.txt'
output_file_name = 'pixels'
width = 512
height = 512
y_fov = np.pi / 3
result_folder = 'pixels'
coef=[1,1,-1.5,1]

def main():
    model_path, proj_folder = parser()
    cam_matrices_path = os.path.join(projection_folder, proj_folder, matrices)
    cams = read_cam_matrices(cam_matrices_path)
    camera_instrincs = pyrender.PerspectiveCamera(yfov=y_fov, aspectRatio=1)
    projection_matrix = camera_instrincs.get_projection_matrix(width, height)
    mesh = trimesh.load(model_path, force='mesh')
    vertices = mesh.vertices
    vertices = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
    for i, cam in enumerate(cams):
        pixels = render_vertices(vertices, projection_matrix, cam)
        # test=apply_dist(pixels)
        # plot(test)
        np.savetxt(os.path.join(projection_folder, proj_folder, result_folder,
                                output_file_name + '_img' + str(i + 1) + '.txt'),
                   pixels, fmt='%i', delimiter=',')
        print(f'file named {output_file_name}_img{str(i + 1)}.txt wrote succesfuly!')
        """

        d_pixels=[]
        for pix in pixels:
            dist_manual=wand_distortion(pix,coef)
            d_pixels.append(dist_manual)
        d_pixels=np.array(d_pixels)
        print(d_pixels)
        """
        d_pixels=apply_and_track_distortion(pixels,coef)
        np.savetxt(os.path.join(projection_folder, proj_folder, result_folder,
                                output_file_name + 'distorted_img' + str(i + 1) + '.txt'),
                   d_pixels, fmt='%i', delimiter=',')
        plot(d_pixels,isIdeal=False)
        plot(pixels)




def apply_and_track_distortion(pixels,coef=(0.5,0.5,0.5,1)):

    tmp = np.zeros((width, height), dtype=np.uint8)
    d_pixels=[]
    for pixel in pixels:
        tmp = np.zeros((width, height), dtype=np.uint8)
        if pixel[0] < 0 or pixel[0] >= width or pixel[1] < 0 or pixel[1] >= height:
            d_pixels.append(np.array([-1,-1]))
        try:
            tmp[int(pixel[1]), int(pixel[0])] = 255
        except Exception:
            continue
        dist=ad.apply_distortion_cv2(tmp,coef)
        i,j=np.where(dist>30)
        if(len(j)==0):
            i,j=np.where(dist>10)
        if len(j)==0:
            print()
        i_avg=i.sum()/len(i)
        j_avg=j.sum()/len(j)
        d_pixels.append(np.array([j_avg,i_avg]))
    return np.array(d_pixels)

    tmp = np.zeros((width, height), dtype=np.uint8)
    d_pixels=[]
    for pixel in pixels:
        tmp = np.zeros((width, height), dtype=np.uint8)
        if pixel[0] < 0 or pixel[0] >= width or pixel[1] < 0 or pixel[1] >= height:
            d_pixels.append(np.array([-1,-1]))
        try:
            tmp[int(pixel[1]), int(pixel[0])] = 255
        except Exception:
            continue
        dist=ad.apply_distortion_cv2(tmp,coef)
        i,j=np.where(dist>30)
        if(len(j)==0):
            i,j=np.where(dist>10)
        if len(j)==0:
            print()
        i_avg=i.sum()/len(i)
        j_avg=j.sum()/len(j)
        nd=reverse_from_pixel(np.array([i_avg,j_avg]),width,height)
        d_radius=(nd[0]**2+nd[1]**2)**0.5
        ni=reverse_from_pixel(pixel,width,height)
        i_radius=(ni[0]**2+ni[1]**2)**0.5
        ratio=d_radius/i_radius
        break
    for pixel in pixels:
        tmp=reverse_from_pixel(pixel,width,height)
        tmp*=ratio
        tmp[0] = (width / 2 * tmp[0] + width / 2)
        tmp[1] = height - (height / 2 * tmp[1] + height / 2)
        d_pixels.append(tmp)

    return np.array(d_pixels)

def plot(pixels,isIdeal=True):
    img = np.zeros((width, height), dtype=np.uint8)
    for pixel in pixels:
        if pixel[0] < 0 or pixel[0] >= width or pixel[1] < 0 or pixel[1] >= height:
            continue
        try:
            img[int(pixel[1]), int(pixel[0])] = 255
        except Exception:
            continue
    if isIdeal:
        d_img=ad.apply_distortion_cv2(img,coef)
        cv2.imshow('ideal', img)
        cv2.imshow('distorted', d_img)
        cv2.waitKey()
    else:
        cv2.imshow('distoted', img)
        cv2.waitKey()



def render_vertices(vertices, proj, cam):
    view = np.vstack((cam, [0, 0, 0, 1]))
    pixels = map_to_pixel(vertices, width, height, proj, view).T
    pixels = pixels[:, [0, 1]]
    # pixels = np.uint8(pixels)
    return pixels


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


def map_to_pixel(point3d, w, h, projection, view):
    p = projection @ (inv(view) @ point3d.T)
    p = p / p[3]
    p[0] = (w / 2 * p[0] + w / 2)
    p[1] = h - (h / 2 * p[1] + h / 2)
    return p


"""
def apply_dist(pixels,coef=(1,1,1,1)):
    tmp=reverse_from_pixel(pixels,width,height).T
    tmp=rdi.applyDistortion(tmp,coef)
    tmp[0] = (width / 2 * tmp[0] + width / 2)
    tmp[1] = height - (height / 2 * tmp[1] + height / 2)
    return tmp.T
"""
def wand_distortion(pixel,coef=(1.5,0.5,0.5,1)):
    """
    copy image wand library barel distortion
    """

    n_pixel=reverse_from_pixel(pixel,512,512)
    radius=(n_pixel[0]**2+n_pixel[1]**2)**(0.5)
    x=coef[0]*np.power(256,0)
    y=coef[1]*np.power(256,0)
    z=coef[2]*np.power(256,0)
    #x=coef[0]
    #y=coef[1]
    #z=coef[2]
    rs=x*np.power(radius,3)+y*np.power(radius,2)+z*radius+coef[3]
    #rs=rs*radius
    res=(1/rs)*n_pixel
    res[0] = (width / 2 * res[0] + width / 2)
    res[1] = height - (height / 2 * res[1] + height / 2)
    return res
    """
    d_pixel=[pixel[0]-256,256-pixel[1]]
    radius=(d_pixel[0]**2+d_pixel[1]**2)**(0.5)
    rscale=1/256
    A=coef[0]*np.power(rscale,3.0)
    B=coef[1]*np.power(rscale,2.0)
    C=coef[2]*rscale
    rs=(A*radius**3)+(B*radius**2)+(C*radius)+coef[3]
    rs*=radius
    res=np.array(pixel)
    res[0]=rs*d_pixel[0]+256
    res[1]=256-rs*d_pixel[1]
    return res
    """



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


if __name__ == '__main__':
    main()
