import numpy as np
import open3d as o3d
import radial_distortion_invariant as rdi
import trimesh
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv



def main():
    model_path='files/iron man.obj'



    #mesh = trimesh.load(model_path, force='mesh', process=False,showcoords=True)
    #mesh.show()
    focal_len=300
    size=512
    K=np.array([[focal_len,0,size/2],[0,focal_len,-size/2],[0,0,1]])
    K_inv=inv(K)
    arr0 = [-10, 4, 0, 0, np.deg2rad(70), 0]
    arr1=[0,6,0,np.deg2rad(-20),0,0]
    arr2 = [-10, 12, 0, 0, np.deg2rad(45), 0]
    arr3 = [-10, 4, 0, 0, np.deg2rad(60), 0]
    arr4=[9,3,0,np.deg2rad(-20),np.deg2rad(280),0]
    arr5=[0,1,7,np.deg2rad(-90),0,0]

    pm=np.array([[1.73205081,0,0,0],[0,1.73205081,0,0],[0,0,-1,-0.1],[0,0,-1,0]])

    cam0=rdi.calc_cam_mat_custom(arr0)
    cam1=rdi.calc_cam_mat_custom(arr1)
    cam2=rdi.calc_cam_mat_custom(arr2)
    cam3=rdi.calc_cam_mat_custom(arr3)
    cam4=rdi.calc_cam_mat_custom(arr4)
    cam5=rdi.calc_cam_mat_custom(arr5)
    point3d=np.array([0.5,5.7,-2.3,1])
    """
    pixels=[]
    pixels.append(cam0@point3d)
    pixels.append(cam1 @ point3d)
    pixels.append(cam2 @ point3d)
    pixels.append(cam3 @ point3d)
    pixels.append(cam4 @ point3d)
    pixels.append(cam5 @ point3d)
    polyCoefs = [0.5, -0.5, 1.5]
    print(pixels)
    pixels=np.array(pixels)
    print(pixels)
    for i in range(pixels.shape[0]):
        pixels[i,:]=pixels[i,:]/pixels[i,2]
    pixels=pixels[:,[0,1]]
    print(pixels)

    l=[cam0,cam1,cam2,cam3,cam4,cam5]
    cams=np.array(l)
    print(cams.shape)
    res=rdi.radialDistortionInvariant3dEstimationMultiview(cams,pixels)
    print('res:',np.array(res))
    """
    """right hand pixels from images"""
    pixelsRight0=np.array([469,203,1])
    pixelsRight1=np.array([383, 115,1])
    pixelsRight2=np.array([348,194,1])
    pixelsRight4 = np.array([255, 284, 1])
    pixelR0=reverse_from_pixel(pixelsRight0,512,512)
    pixelR1 = reverse_from_pixel(pixelsRight1, 512, 512)
    pixelR4 = reverse_from_pixel(pixelsRight4, 512, 512)
    """left hand pixels from images"""
    pixelsLeft0=np.array([400,275,1])
    pixelsLeft1=np.array([384,392,1])
    pixelsLeft2=np.array([330, 260,1])
    pixelsLeft4=np.array([334, 345,1])

    pixelL0=reverse_from_pixel(pixelsLeft0,512,512)
    pixelL1 = reverse_from_pixel(pixelsLeft1, 512, 512)
    pixelL4 = reverse_from_pixel(pixelsLeft4, 512, 512)


    pixelHead0=np.array([239,226,1])
    pixelHead1 = np.array([255, 27, 1])
    pixelHead4 = np.array([330, 53, 1])
    ph0=reverse_pixel(pixelHead0,512,512)
    ph1 = reverse_pixel(pixelHead1, 512, 512)
    ph4 = reverse_pixel(pixelHead4, 512, 512)

    resRight,ok=rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam0), inv_cam_mat(cam1),inv_cam_mat( cam4),pixelR0,pixelR1,pixelR4)
    print('right hand 3d:\n',resRight)
    print(ok)

    resLeft, ok = rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam0), inv_cam_mat(cam1),inv_cam_mat( cam4),pixelL0,pixelL1,pixelL4)
    print('left hand 3d:\n', resLeft)
    print(ok)

    #resHead,ok=rdi.radialDistortionInvariant3dEstimation(cam0,cam1,cam4,ph0,ph1,ph4)
    #print('head:\n',resHead)
    return




def reverse_pixel(pixel,width,height):
    rev=np.float32([1,1,1])
    rev[0]=pixel[0]*(1/width)
    rev[1]=pixel[1]*(1/height)
    rev-=0.5
    rev*=2
    return rev

def reverse_from_pixel(pixels,w,h):
    p=np.array(pixels)
    if(len(pixels.shape)==1):
        p[0]=((2*pixels[0])/w)-1
        p[1]=((-2*pixels[1])/h)+1
        return p
    for i,pix in enumerate(pixels):
        p[i,0]=((2*pix[0])/w)-1
        p[i,1]=((-2*pix[1])/h)+1
    return p

def inv_cam_mat(cam):
    if(cam.shape==(4,4)):
        inverse=inv(cam)
        return inverse[0:3]
    inverse=np.vstack((cam,[0,0,0,1]))
    return inv(inverse)[0:3]


if __name__=='__main__':
    main()