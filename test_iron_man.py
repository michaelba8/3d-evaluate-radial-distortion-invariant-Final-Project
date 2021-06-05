import numpy as np
import radial_distortion_invariant as rdi
import matplotlib.pyplot as plt
import solve_projection as sv
import trimesh
import pyrender
import cv2
import os
from numpy.linalg import inv
import point_estimation as pe

def main():
    cams=get_cameras()
    p0=np.array([246,224])
    p1=np.array([256,27])
    p2=np.array([217,234])
    p3=np.array([319,223])
    p4=np.array([328,352])
    p5=np.array([255,198])
    p6=np.array([255,238])
    p7=np.array([144,239])
    ideal_pixels=np.array([p0,p1,p2,p3,p4,p5,p6,p7])

    p0=np.array([247,226])
    p1=np.array([256,84])
    p2=np.array([218,234])
    p3=np.array([313,226])
    p4=np.array([307,101])
    p5=np.array([255,203])
    p6=np.array([255,239])
    p7=np.array([160,240])
    distorted_pixels=np.array([p0,p1,p2,p3,p4,p5,p6,p7])

    p123,ok=pe.estimate_3d_point(cams[1],cams[2],cams[3],p1,p2,p3)
    print(p123,ok)
    res,ok=pe.estimate_3d_point_mv(cams[1:6],ideal_pixels[1:6])
    print(res,ok)
    model_path='files/iron man.obj'
    mesh=trimesh.load(model_path,force='mesh',process=False)
    print()
    return
    p124,ok=pe.estimate_3d_point(cam1,cam2,cam4,p1,p2,p4)
    p134,ok=pe.estimate_3d_point(cam1,cam3,cam4,p1,p3,p4)
    p234,ok=pe.estimate_3d_point(cam2,cam3,cam4,p2,p3,p4)
    p=np.array([[-1.5],[0.5],[2.5],[1.0]])

    print('images 1,2,3:\n',p123,'\nnorm of the error:',np.linalg.norm(p123-p))
    print('images 1,2,4:\n',p124,'\nnorm of the error:',np.linalg.norm(p124-p))
    print('images 1,3,4:\n',p134,'\nnorm of the error:',np.linalg.norm(p134-p))
    print('images 2,3,4:\n',p234,'\nnorm of the error:',np.linalg.norm(p234-p))
    print()

    cams=np.array([cam1,cam2,cam3,cam4,cam5,cam6])
    pixels=np.array([p1,p2,p3,p4,p5,p6])
    res,ok=pe.estimate_3d_point_mv(cams,pixels)
    print('images 1,2,3,4,5,6 multiview (distorted pixels) :\n',res,'\nnorm of the error:',np.linalg.norm(res-p))
    print()

    model_path='files/iron man.obj'
    mesh=trimesh.load(model_path,force='mesh',process=False)

    return

    rp1=reverse_from_pixel(p1,512,512)
    rp2=reverse_from_pixel(p2,512,512)
    rp3=reverse_from_pixel(p3,512,512)
    rp4=reverse_from_pixel(p4,512,512)
    print(rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam2),inv_cam_mat(cam1),inv_cam_mat(cam3),rp2,rp1,rp3))


    print('\ndistorted pixels:\n',p3d)
    print('\nperfect pixels:\n',r_p3)
    print('\nestimated points:\n')
    for i in range(r_p4.shape[0]):
        res,ok=rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam2t),inv_cam_mat(cam3t),inv_cam_mat(cam4t),p2d[i],p3d[i],p4d[i])
        res=np.array(res)
        print(str(i)+",\n"+str(res)+'\n')



    Tx = 3  # verticle axis
    Ty = 4  # horizontle axis
    Tz = 10  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(-15)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam5=rdi.calc_cam_mat_custom(arr)
    cam5t=np.vstack((cam5,[0,0,0,1]))

    vertices=map_to_pixel(ver,512,512,projection,cam5t).T
    img=np.zeros((512,512),dtype=np.uint8)
    for pixel in vertices:
        try:
            img[int(pixel[1]),int(pixel[0])]=255
        except:
            continue
    img=cv2.dilate(img,np.ones((3,3)))
    cv2.imshow('sd',img)
    cv2.waitKey()
    print(vertices)
    t=os.path.join(cur_dir,folder_result)

    if(to_save):
        os.chdir(os.path.join(cur_dir,folder_result))
        cv2.imwrite(image_name,img)
    print()




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



def get_cameras():
    """cam0"""
    Tx=-10 #verticle axis
    Ty=4 #horizontle axis
    Tz=0 #height axis
    Rx=np.deg2rad(0)
    Ry=np.deg2rad(70)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam0 = rdi.calc_cam_mat_custom(arr)

    """cam1"""
    Tx=0 #verticle axis
    Ty=6 #horizontle axis
    Tz=0 #height axis
    Rx=np.deg2rad(-20)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam1 = rdi.calc_cam_mat_custom(arr)

    """cam2"""
    Tx = -10  # verticle axis
    Ty = 12  # horizontle axis
    Tz = 0  # height axis
    Rx = np.deg2rad(0)
    Ry = np.deg2rad(45)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam2 = rdi.calc_cam_mat_custom(arr)

    """cam3"""
    Tx = -10  # verticle axis
    Ty = 4  # horizontle axis
    Tz =0  # height axis
    Rx = np.deg2rad(0)
    Ry = np.deg2rad(60)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam3 = rdi.calc_cam_mat_custom(arr)

    """cam4"""
    Tx=9 #verticle axis
    Ty=3 #horizontle axis
    Tz=0 #height axis
    Rx=np.deg2rad(-20)
    Ry=np.deg2rad(280)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam4 = rdi.calc_cam_mat_custom(arr)

    """cam5"""
    Tx=0 #verticle axis
    Ty=1 #horizontle axis
    Tz=7 #height axis
    Rx=np.deg2rad(-90)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam5 = rdi.calc_cam_mat_custom(arr)

    """cam6"""
    Tx=0 #verticle axis
    Ty=20 #horizontle axis
    Tz=0 #height axis
    Rx=np.deg2rad(0)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam6 = rdi.calc_cam_mat_custom(arr)

    """cam7"""
    Tx=5 #verticle axis
    Ty=20 #horizontle axis
    Tz=0 #height axis
    Rx=np.deg2rad(0)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam7 = rdi.calc_cam_mat_custom(arr)

    return np.array([cam0,cam1,cam2,cam3,cam4,cam5,cam6,cam7])

def inv_cam_mat(cam):
    if(cam.shape==(4,4)):
        inverse=inv(cam)
        return inverse[0:3]
    inverse=np.vstack((cam,[0,0,0,1]))
    return inv(inverse)[0:3]

if __name__=="__main__":
    main()

