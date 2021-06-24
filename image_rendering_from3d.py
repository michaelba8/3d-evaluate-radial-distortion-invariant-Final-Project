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
    image_name='image2.png'
    model_path='files/deer.obj'
    folder_result = "projection results/deer"
    cur_dir=os.path.dirname(__file__)
    temp=trimesh.load(model_path,force='mesh',process=False)
    mesh = pyrender.Mesh.from_trimesh(temp ,smooth=False)
    scene = pyrender.Scene(ambient_light=[.1, .1, .3], bg_color=[0, 0, 0])

    camera = pyrender.PerspectiveCamera(yfov=np.pi/3 ,aspectRatio=1)
    light = pyrender.DirectionalLight(color=[1, 1, 1], intensity=500)

    Tx=0 #verticle axis
    Ty=150 #horizontle axis
    Tz=250 #height axis
    Rx=np.deg2rad(0)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    mat=rdi.calc_cam_mat_custom(arr)
    mat=np.vstack((mat,[0,0,0,1]))
    T=np.hstack((np.eye(3),np.array([mesh.centroid]).T))
    T=np.vstack((T,[0,0,0,1]))
    print(inv(T))
    scene.add(mesh, pose=np.eye(4))
    scene.add(light, pose=mat)
    scene.add(camera, pose=mat)
    # render scene
    r = pyrender.OffscreenRenderer(512, 512,point_size=1)
    color, _ = r.render(scene)
    title='Tx='+str(Tx)+', Ty='+str(Ty)+', Tz='+str(Tz)+', Rx='+str(np.rad2deg(Rx))+ ' deg, Ry=' + str(np.rad2deg(Ry)) + ' deg, Rz=' +str(np.rad2deg(Rz))+ ' deg'

    cv2.imshow('sds',color)
    cv2.waitKey()
    t=os.path.join(cur_dir,folder_result)
    os.chdir(os.path.join(cur_dir,folder_result))
    if(to_save):
        cv2.imwrite(image_name,color)


    if(to_save):
        f = open(t+"/cam matrices.txt", "a")
        f.write(image_name)
        f.write(':  ')
        f.write(title)
        f.write('\n')
        f.close()
    print(camera.get_projection_matrix())



def read_images(folder):
    """Reading all the image from folder"""
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(filename,0)
        if img is not None:
            images.append(img)

    return images

def map_to_pixel(point3d,w,h,projection,view):
    p=projection@inv(view)@point3d.T
    p=p/p[3]
    p[0]=(w/2*p[0]+w/2)
    p[1]=h-(h/2*p[1]+h/2)
    return p

if __name__=='__main__':
   main()
