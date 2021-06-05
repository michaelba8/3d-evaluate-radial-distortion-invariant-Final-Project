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

    to_save=False
    image_name='image6.png'
    model_path='files/deer.obj'
    folder_result = "projection results\\test2"
    cur_dir=os.path.dirname(__file__)
    zfar=100
    znear=0.1
    focal_len=300
    size=512
    temp=trimesh.load(model_path,process=False)
    #temp=trimesh.creation.box((3,1,5))
    mesh = pyrender.Mesh.from_trimesh(temp, smooth=False)
    scene = pyrender.Scene(ambient_light=[.1, .1, .3], bg_color=[0, 0, 0])
    #pm=np.array([[2* focal_len /size,0,0,0],[0,-2*focal_len/size,0,0],[0,0,(zfar+znear)/(zfar-znear),-1],[0,0,2*(zfar+znear)/(zfar-znear),0]])
    alpha=np.pi/4
    camera = pyrender.PerspectiveCamera(yfov=alpha,zfar=zfar ,aspectRatio=1)
    #camera = pyrender.IntrinsicsCamera(focal_len,focal_len,0,0,znear=znear,zfar=zfar)
    light = pyrender.DirectionalLight(color=[1, 1, 1], intensity=500)

    point3d=np.array([1,1.5,-2,1]).T

    """cam1"""
    Tx=95 #verticle axis
    Ty=-270 #horizontle axis
    Tz=130 #height axis
    Rx=np.deg2rad(-10)
    Ry=np.deg2rad(210)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam1 = rdi.calc_cam_mat_custom(arr)
    cam1t=np.vstack((cam1,[0,0,0,1]))

    mat=rdi.calc_cam_mat_custom(arr)
    mat=np.vstack((mat,[0,0,0,1]))
    point3d=np.array([1,1.5,-2,1]).T
    scene.add(mesh, pose=np.eye(4))
    scene.add(light, pose=cam1t)
    scene.add(camera, pose=cam1t)
    test=camera.get_projection_matrix(size,size)
   #pyrender.viewer.Viewer(scene)

    # render scene
    r = pyrender.OffscreenRenderer(size, size)
    color, _ = r.render(scene)
    title='Tx='+str(Tx)+', Ty='+str(Ty)+', Tz='+str(Tz)+', Rx='+str(np.rad2deg(Rx))+ ' deg, Ry=' + str(np.rad2deg(Ry)) + ' deg, Rz=' +str(np.rad2deg(Rz))+ ' deg'
    cv2.imshow('sds',color)
    cv2.waitKey()
    t=os.path.join(cur_dir,folder_result)
    os.chdir(os.path.join(cur_dir,folder_result))
    if(to_save):
        cv2.imwrite(image_name,color)

    plt.figure(figsize=(8, 8)),plt.imshow(color);
    plt.suptitle(title)
    plt.show()

    if(to_save):
        f = open(t+"/cam matrices.txt", "a")
        f.write(image_name)
        f.write(':  ')
        f.write(title)
        f.write('\n')
        f.close()




def read_images(folder):
    """Reading all the image from folder"""
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(filename,0)
        if img is not None:
            images.append(img)

    return images


def map_to_pixel(point3d,w,h,projection,view):
    p=projection@view@point3d
    p=p/p[3]
    p[0]=w/2*p[0]+w/2
    p[1]=h/2*p[1]+h/2
    return p

def reverse_from_pixel(pixel,w,h,projection,view):
    p=np.array(pixel)
    p[0]=((2*pixel[0])/w)-1
    p[1]=h-((2*pixel[1])/h)-1
    return p



if __name__=='__main__':
   main()
