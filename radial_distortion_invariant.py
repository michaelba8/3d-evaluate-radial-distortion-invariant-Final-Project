import numpy as np
import numpy.linalg as la






def getPlanesFromPixel(cam,pixel_distorted):
    """ WORKING!!!
        need more tests
        get an input of pixel(x,y) and camera matrix and return plane"""
    p0=np.array([[0],[0],[0]])
    p1=np.array([[0],[0],[1]])
    p2=np.array([[pixel_distorted[0]],[pixel_distorted[1]],[1]])
    R=cam[:,0:3]
    T=cam[:,[3]]
    Rt=R.transpose()
    p0=np.matmul(Rt,p0-T)
    p1 = np.matmul(Rt, p1 - T)
    p2 = np.matmul(Rt, p2 - T)
    A=np.vstack((p0.transpose(),p1.transpose(),p2.transpose()))
    A=np.hstack((A,np.ones((3,1))))
    u,s,v=la.svd(A)
    ok=s[2]>10**(-5)
    plane=v.transpose()[:,-1]
    return plane,ok

def calc_cam_mat(cam_params):
    """
    get camera paramaters as a list with size=6
    and return camera matrix 3X4
    """
    az = cam_params[5]
    ro = cam_params[4]
    pi = cam_params[3]
    az_mat=np.array([[np.cos(az),-1*np.sin(az),0],[np.sin(az),np.cos(az),0],[0,0,0]])
    ro_mat=np.array([[np.cos(ro),0,-1*np.sin(ro)],[0,0,0],[np.sin(ro),0,np.cos(ro)]])
    pi_mat=np.array([[0,0,0],[0,np.cos(pi),-1*np.sin(pi)],[0,np.sin(pi),np.cos(pi)]])
    R=ro_mat*pi_mat*az_mat
    R=np.matmul(np.matmul(ro_mat,pi_mat),az_mat)
    cam_params=np.array([cam_params])
    t=cam_params[:,0:3].transpose()
    cam_mat=np.hstack((R,t))
    return  cam_mat

def applyDistortion(idealPixels,polyCoefs):
    """create distortion"""
    ideal_pixel_radius=(idealPixels[0,:]**2+idealPixels[1,:]**2)**(0.5)

    distorted_pixel_radius=polyCoefs[0]*np.power(ideal_pixel_radius,5) +polyCoefs[1]*np.power(ideal_pixel_radius,3)+ideal_pixel_radius*polyCoefs[2]
    print("dsr: \n",distorted_pixel_radius)
    cam_radius_ratio=distorted_pixel_radius/ideal_pixel_radius
    first=idealPixels[0,:]*cam_radius_ratio
    second=idealPixels[1,:]*cam_radius_ratio
    distorted_pixels=np.vstack((first,second))
    return distorted_pixels





def getRandomCamera(T):
    pass

def calc_points_3d(points1,points2,points3):
    pass

def tl2cen(points, size):
    """
    WORK!!!
    dont know what this function doing
    """
    hsz=size/2
    points[:,0]=points[:,0]-hsz[0]
    points[:, 1] =  hsz[1]-points[:, 1]
    return points

def estimationMultiview3d(cams,camsPixelDistorted):
    """
    this function takes 3 pixels of the same 3D point and the 3 camera
    matrices, and calculates the 3d point location, even under severe radial distortion.
    success - false if two cameras are identical or if all 3 cameras center
    ray is looking at the same 3D point or are parallel.
    """
    pass

def get3planesIntersectionMultiview(planes):
    pass

def radialDistortionInvariant3dEstimation(cam0,cam1,cam2,cam0pixelDistorted,cam1pixelDistorted,cam2pixelDistorted):
    """
    this function takes 3 pixels of the same 3D point and the 3 camera
    matrices, and calculates the 3d point location, even under severe radial distortion.
    success - false if two cameras are identical or if all 3 cameras center
    ray is looking at the same 3D point or are parallel.
    :param cam0:
    :param cam1:
    :param cam2:
    :param cam0pixelDistorted:
    :param cam1pixelDistorted:
    :param cam2pixelDistorted:
    :return:
    """
    pass

def get3planesIntersection(plane0,plane1,plane2):
    pass



def make_of_from_box_corners():
    pass

def  radialDistortionInvariant3dEstimationMultiview(cams,camsPixelDistorted):
    """
    this function takes 3 pixels of the same 3D point and the 3 camera
    matrices, and calculates the 3d point location, even under severe radial distortion.
    success - false if two cameras are identical or if all 3 cameras center
    ray is looking at the same 3D point or are parallel.
    :param cams:
    :param camsPixelDistorted:
    :return:
    """

    pass
