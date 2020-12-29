import numpy as np
import numpy.linalg as la
import radial_distortion_invariant as rdi





def main():
    cam = np.array([-0.7152, 1.4076, 1.0733, - 0.09384593045608725, - 2.2928236739615913, 2.6677269190232535])
    mat_cam=rdi.calc_cam_mat(cam)
    dp=[10,10]
    plane,ok=rdi.getPlanesFromPixel(mat_cam,dp)
    print(plane)
    print(ok)
    print("test123")

if __name__=="__main__":
    main()