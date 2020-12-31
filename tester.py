import numpy as np
import numpy.linalg as la
import radial_distortion_invariant as rdi
import cv2





def main():
    """cam = np.array([-0.7152, 1.4076, 1.0733, - 0.09384593045608725, - 2.2928236739615913, 2.6677269190232535])
    mat_cam=rdi.calc_cam_mat(cam)
    dp=[10,10]
    plane,ok=rdi.getPlanesFromPixel(mat_cam,dp)
    ideal_pixels=np.array([[-0.1320 ,  -0.0742,    0.0744  ,  0.0732  , -0.0326   , 0.2429  , -0.1220 ,  -0.1036   , 0.1104  ,  0.1224,
               0.1697, - 0.0121, - 0.1603,- 0.0538 ,- 0.0412 ,   0.0083 ,- 0.0323 ,- 0.1555   , 0.0530, - 0.0995],
              [-0.4958   ,-0.0279 ,  -0.1094  , -0.1945   ,-0.1591  , -0.4297,   -0.1864  , -0.1687   ,-0.4285   ,-0.1095,
               -0.1659 ,  -0.1418,   -0.1547 ,  -0.2818,   -0.3602 ,  -0.4137 ,  -0.2166 ,  -0.2116 ,  -0.1041 ,  -0.2789]])
    polyCoefs=[0.5,-0.5,1.5]
    #test=rdi.applyDistortion(ideal_pixels,polyCoefs)
    mat=np.array([[235,53],[497,32],[612,99],[305,149],[236,231],[289,337],[531,267]])
    rdi.get3planesIntersectionMultiview()"""
    cam0=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    a=0.17
    cam1=np.array([[1,0,0,0],[0,np.cos(a),np.sin(a),-1],[0,-np.sin(a),np.cos(a),0]])
    cam2=np.array([[np.cos(a),0,np.sin(a),-1],[0,1,0,0],[-np.sin(a),0,np.cos(a),0]])
    dp0=np.array([3,2])
    dp1 = np.array([4, 5])
    dp2 = np.array([6, 6])
    plane0,ok0=rdi.getPlanesFromPixel(cam0,dp0)
    plane1,ok1 = rdi.getPlanesFromPixel(cam1, dp1)
    plane2 ,ok2= rdi.getPlanesFromPixel(cam2, dp2)
    print("plane0: \n",plane0)
    print("plane1: \n", plane1)
    print("plane2: \n", plane2)
    point,ok=rdi.get3planesIntersection(plane0,plane1,plane2)
    print(point,ok)








if __name__=="__main__":
    main()