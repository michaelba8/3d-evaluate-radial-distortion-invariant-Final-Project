import numpy as np
import numpy.linalg as la
import radial_distortion_invariant as rdi

def main():
    """
    synthetic simulation of 3d points estimation using the same random inputs as the matlab code
    as you can see the estimation error of the matlab code and the python code are both in the same scale O(e^-13)

    usage:
        for using this python simulation, you need to run the matlab code from the directory and then run this code
        otherwise the simulation will not run
    """
    matlabEstimation=matrix_from_file('Matlab Code/estimatedPoints.txt')
    cam0=matrix_from_file('Matlab Code/cam0.txt')
    cam1=matrix_from_file('Matlab Code/cam1.txt')
    cam2 = matrix_from_file('Matlab Code/cam2.txt')
    idealPixeelCam0=matrix_from_file('Matlab Code/cam0pixelsIdeal.txt')
    idealPixeelCam1 = matrix_from_file('Matlab Code/cam1pixelsIdeal.txt')
    idealPixeelCam2 = matrix_from_file('Matlab Code/cam2pixelsIdeal.txt')
    points3D=matrix_from_file('Matlab Code/points3D.txt')
    polyCoefs = [1000, 350, 12323]
    polyCoefs = [0.5, -0.5, 1.5]
    print(points3D.shape)
    distortedPixelCam0=np.array(rdi.applyDistortion(idealPixeelCam0,polyCoefs))
    distortedPixelCam1=np.array(rdi.applyDistortion(idealPixeelCam1,polyCoefs))
    distortedPixelCam2=np.array(rdi.applyDistortion(idealPixeelCam2,polyCoefs))
    mat,ok=rdi.radialDistortionInvariant3dEstimation(cam0,cam1,cam2,distortedPixelCam0[:,0],distortedPixelCam1[:,0],distortedPixelCam2[:,0])
    for i in range(1,idealPixeelCam2.shape[1],1):
        res,ok=rdi.radialDistortionInvariant3dEstimation(cam0,cam1,cam2,distortedPixelCam0[:,i],distortedPixelCam1[:,i],distortedPixelCam2[:,i])
        mat=np.hstack((mat,res))
    errP=mat-points3D
    errM=matlabEstimation-points3D
    print("python ",la.norm(errP))
    print("Matlab " ,la.norm(errM))
    print(errP.shape)



def matrix_from_file(file_name):
    with open(file_name, 'r') as f:
        l = [[np.float64(num) for num in line.split(',')] for line in f]
        f.close()
    return np.array(l,dtype=np.float64)













if __name__=="__main__":
    main()