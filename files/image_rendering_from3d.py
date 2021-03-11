import numpy as np
import radial_distortion_invariant as rdi
import matplotlib.pyplot as plt
import trimesh
import pyrender

def main():
    """in this file we load 3d model and project 3 different images from 3 different views
       the images need to be saved manually because finding the perfect view might take few tries
       the images render using Camera matrix just like in the algorithm using closed function so
       we can use the images for testing later"""
    temp=trimesh.load('files/car2.obj',force='mesh',process=False)
    mesh = pyrender.Mesh.from_trimesh(temp, smooth=False)
    scene = pyrender.Scene(ambient_light=[.1, .1, .3], bg_color=[0, 0, 0])

    camera = pyrender.PerspectiveCamera(yfov=np.pi/3 ,aspectRatio=1)
    light = pyrender.DirectionalLight(color=[1, 1, 1], intensity=500)
    c = 2 ** -0.5
    scene.add(mesh, pose=np.eye(4))

    Tx=-22 #verticle axis
    Ty=-2 #horizontle axis
    Tz=5 #height axis
    Rx=np.pi/12
    Ry=np.deg2rad(100)
    Rz=0
    arr=[Tx,Tz,Ty,Rx,Ry,Rz]
    mat=rdi.calc_cam_mat_custom(arr)
    mat=np.vstack((mat,[0,0,0,1]))
    print(mat)
    scene.add(light, pose=mat)
    scene.add(camera, pose=mat)
    # render scene
    r = pyrender.OffscreenRenderer(512, 512)
    color, _ = r.render(scene)
    title='Tx='+str(Tx)+', Ty='+str(Ty)+', Tz='+str(Tz)+', Rx='+str(Rx/np.pi)+ '*PI, Ry=' + str(Ry/np.pi) + '*PI, Rz=' +str(Rz/np.pi)+ '*PI'
    plt.figure(figsize=(8, 8)),plt.imshow(color);
    plt.suptitle(title)
    plt.show()

if __name__=='__main__':
   main()
