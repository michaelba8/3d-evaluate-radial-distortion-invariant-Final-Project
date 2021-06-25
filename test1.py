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
    cur_dir=os.path.dirname(__file__)
    to_save=False
    image_name='image3.png'
    folder_result = "projection results\\test1"
    size=512
    mesh=trimesh.creation.box((3,1,5))
    ver=np.hstack((mesh.vertices,np.ones((8,1))))
    print(ver)
    top_left=ver[3]
    #print('cube vertices:\n',ver)
    cam1,cam2,cam3,cam4,cam5,cam6=get_cameras()
    cam1t=np.vstack((cam1,[0,0,0,1]))
    cam2t=np.vstack((cam2,[0,0,0,1]))
    cam3t=np.vstack((cam3,[0,0,0,1]))
    cam4t=np.vstack((cam4,[0,0,0,1]))

    projection=get_projection_matrix()

    coef=[0.0000005,-0.0000005,0.0000015]
    p3=map_to_pixel(ver,512,512,projection,cam3t).T
    p2=map_to_pixel(ver,512,512,projection,cam2t).T
    p4=map_to_pixel(ver,512,512,projection,cam4t).T

    r_p3=reverse_from_pixel(p3,512,512)
    r_p2=reverse_from_pixel(p2,512,512)
    r_p4=reverse_from_pixel(p4,512,512)
    
    p3d=rdi.applyDistortionSingleP(r_p3.T,coef).T
    p2d=rdi.applyDistortionSingleP(r_p2.T,coef).T
    p4d=rdi.applyDistortionSingleP(r_p4.T,coef).T

    """ideal pixesls"""
    p1=np.array([496,248])
    p2=np.array([91,338])
    p3=np.array([482,278])
    p4=np.array([92,352])
    p5=np.array([223,483])
    p6=np.array([56,273])

    """distorted pixels"""
    p1=np.array([408,252])
    p2=np.array([141,313])
    p3=np.array([401,271])
    p4=np.array([144,322])
    p5=np.array([234,402])
    p6=np.array([120,268])

    p123,ok=pe.estimate_3d_point(cam1,cam2,cam3,p1,p2,p3)
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


    #print('\ndistorted pixels:\n',p3d)
    #print('\nperfect pixels:\n',r_p3)
    print('\nestimated points:\n')
    for i in range(r_p4.shape[0]):
        res,ok=rdi.radialDistortionInvariant3dEstimation(inv_cam_mat(cam2t),inv_cam_mat(cam3t),inv_cam_mat(cam4t),p2d[i],p3d[i],p4d[i])
        res=np.array(res)
        print(str(i)+",\n"+str(res)+'\n')


    vertices=map_to_pixel(ver,512,512,projection,cam2t).T

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
    p=projection@(inv(view)@point3d.T)
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
    """cam1"""
    Tx=-3 #verticle axis
    Ty=3 #horizontle axis
    Tz=-8 #height axis
    Rx=np.deg2rad(-15)
    Ry=np.deg2rad(150)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam1 = rdi.calc_cam_mat_custom(arr)
    cam1t=np.vstack((cam1,[0,0,0,1]))

    """cam2"""
    Tx = 3  # verticle axis
    Ty = 4  # horizontle axis
    Tz = 10  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(-15)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam2 = rdi.calc_cam_mat_custom(arr)

    """cam3"""
    Tx = -9  # verticle axis
    Ty = 3  # horizontle axis
    Tz =-2  # height axis
    Rx = np.deg2rad(-15)
    Ry = np.deg2rad(100)
    Rz = np.deg2rad(0)
    arr = [Tx, Ty, Tz, Rx, Ry, Rz]
    cam3 = rdi.calc_cam_mat_custom(arr)
    cam3t=np.vstack((cam3,[0,0,0,1]))

    """cam4"""
    Tx=1 #verticle axis
    Ty=10 #horizontle axis
    Tz=1 #height axis
    Rx=np.deg2rad(-90)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam4 = rdi.calc_cam_mat_custom(arr)

    """cam5"""
    Tx=-1 #verticle axis
    Ty=10 #horizontle axis
    Tz=-1 #height axis
    Rx=np.deg2rad(-90)
    Ry=np.deg2rad(0)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam5 = rdi.calc_cam_mat_custom(arr)

    """cam6"""
    Tx=7 #verticle axis
    Ty=3 #horizontle axis
    Tz=5 #height axis
    Rx=np.deg2rad(-15)
    Ry=np.deg2rad(-55)
    Rz=np.deg2rad(0)
    arr=[Tx,Ty,Tz,Rx,Ry,Rz]
    cam6 = rdi.calc_cam_mat_custom(arr)

    return cam1,cam2,cam3,cam4,cam5,cam6

def inv_cam_mat(cam):
    if(cam.shape==(4,4)):
        inverse=inv(cam)
        return inverse[0:3]
    inverse=np.vstack((cam,[0,0,0,1]))
    return inv(inverse)[0:3]
def get_projection_matrix():
    return np.array(
                   [[ 1.73205081,  0.        ,  0.        ,  0.        ],
                    [ 0.        ,  1.73205081,  0.        ,  0.        ],
                    [ 0.        ,  0.        , -1.        , -0.1       ],
                    [ 0.        ,  0.        , -1.        ,  0.        ]])



if __name__=="__main__":
    main()

