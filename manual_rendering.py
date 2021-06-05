import numpy as np
import radial_distortion_invariant as rdi
import matplotlib.pyplot as plt
import trimesh
import pyrender
import cv2
import os
from numpy.linalg import inv


def main():
    """in this file we load 3d model and project 3 or more different images from multiply different views
       some attributes at the start of the code have to be initalised:
       #image_name= the output image name if saved
       #model_path= the path of the model file we project
       folder_results= the folder that the results will be saved on
       #to_save (boolean):  if False the program will only show the resulted image but will not save anything (for debugging)

       in the folder results we also write the camera matrices into a text file called 'cam matices.txt'

       the images render using Camera matrix just like in the algorithm using closed function so
       we can use the images for testing later"""

    to_save = False
    image_name = 'image6.png'
    model_path = 'files/deer.obj'
    folder_result = "projection results/test2"
    cur_dir = os.path.dirname(__file__)
    zfar = 100
    size = 512
    temp=trimesh.load(model_path,process=False)
    #temp = trimesh.creation.box((2, 3, 1))
    #temp.apply_translation([-1, -2, 1])
    mesh = pyrender.Mesh.from_trimesh(temp, smooth=False)
    scene = pyrender.Scene(ambient_light=[.1, .1, .3], bg_color=[0, 0, 0])
    alpha = np.pi / 3
    camera = pyrender.PerspectiveCamera(yfov=alpha, aspectRatio=1)
    light = pyrender.DirectionalLight(color=[1, 1, 1], intensity=500)

    """cam1"""
    Tx = 200  # verticle axis
    Ty = 250  # horizontle axis
    Tz = 350  # height axis
    Rx = np.deg2rad(-25)
    Ry = np.deg2rad(-20)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam1 = rdi.calc_cam_mat_custom(arr)
    cam1t = np.vstack((cam1, [0, 0, 0, 1]))

    scene.add(mesh, pose=np.eye(4))
    scene.add(light, pose=cam1t)
    scene.add(camera, pose=cam1t)
    proj = camera.get_projection_matrix(size, size)
    r = pyrender.OffscreenRenderer(size, size,point_size=1)
    color, _ = r.render(scene)
    title = 'Tx=' + str(Tx) + ', Ty=' + str(Ty) + ', Tz=' + str(Tz) + ', Rx=' + str(np.rad2deg(Rx)) + ' deg, Ry=' + str(
        np.rad2deg(Ry)) + ' deg, Rz=' + str(np.rad2deg(Rz)) + ' deg'

    vertices = temp.vertices
    vertices = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
    sim = mark_corners(vertices, proj, cam1t, color)
    cv2.imshow('sim', sim)
    cv2.waitKey()
    t = os.path.join(cur_dir, folder_result)
    os.chdir(os.path.join(cur_dir, folder_result))
    if (to_save):
        cv2.imwrite(image_name, color)

    if (to_save):
        f = open(t + "/cam matrices.txt", "a")
        f.write(image_name)
        f.write(':  ')
        f.write(title)
        f.write('\n')
        f.close()


def simulate_rendering(ver, projection, view):
    vertices = map_to_pixel(ver, 512, 512, projection, view).T
    img = np.zeros((512, 512), dtype=np.uint8)
    for pixel in vertices:
        if(pixel[0]<0 or pixel[0]>=512 or pixel[1]<0 or pixel[1]>=512):
            continue
        try:
            img[int(pixel[1]), int(pixel[0])] = 255
        except:
            continue
    img = cv2.dilate(img, np.ones((3, 3)))
    return img


def mark_corners(ver, projection, view, org_img):
    img = simulate_rendering(ver, projection, view)
    res = np.array(org_img)
    for i in range(res.shape[0]):
        for j in range(res.shape[1]):
            if img[i, j] != 0:
                res[i, j] = [0, 0, 255]
    return res


def read_images(folder):
    """Reading all the image from folder"""
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(filename, 0)
        if img is not None:
            images.append(img)

    return images


def map_to_pixel(point3d, w, h, projection, view):
    p = projection @ (inv(view) @ point3d.T)
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
        p[0,i] = ((2 * pix[0]) / w) - 1
        p[1,i] = ((-2 * pix[1]) / h) + 1

    return p


if __name__ == '__main__':
    main()
