import radial_distortion_invariant as rdi
import numpy as np
from numpy.linalg import inv
import cv2




def main():
    pass





def estimate_3d_point(cam1,cam2,cam3,pixel1,pixel2,pixel3,width=512,height=512):
    rp1=reverse_from_pixel(pixel1,width,height)
    rp2=reverse_from_pixel(pixel2,width,height)
    rp3=reverse_from_pixel(pixel3,width,height)
    res,ok=rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam1),inv_cam_mat(cam2),inv_cam_mat(cam3),rp1,rp2,rp3)
    return res,ok

def inv_cam_mat(cam):
    if(cam.shape==(4,4)):
        inverse=inv(cam)
        return inverse[0:3]
    inverse=np.vstack((cam,[0,0,0,1]))
    return inv(inverse)[0:3]


def map_to_pixel(point3d,w,h,projection,view):
    p=projection@inv(view)@point3d.T
    p=p/p[3]
    p[0]=(w/2*p[0]+w/2)
    p[1]=h-(h/2*p[1]+h/2)
    return p


def reverse_from_pixel(pixels,w,h):
    p=np.float32(pixels)
    if(len(pixels.shape)==1):
        p[0]=((2*pixels[0])/w)-1
        p[1]=((-2*pixels[1])/h)+1
        return p
    for i,pix in enumerate(pixels):
        p[i,0]=((2*pix[0])/w)-1
        p[i,1]=((-2*pix[1])/h)+1
    return p


if __name__=='__main__':
    main()