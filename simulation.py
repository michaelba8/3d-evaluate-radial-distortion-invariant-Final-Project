import numpy as np
import numpy.linalg as la
import radial_distortion_invariant as rdi

def main():
    cam0=matrix_from_file('cam0.txt')
    cam1=matrix_from_file('cam1.txt')
    cam2 = matrix_from_file('cam2.txt')
    idealPixeelCam0=matrix_from_file('cam0pixelsIdeal.txt')
    idealPixeelCam1 = matrix_from_file('cam0pixelsIdeal.txt')
    idealPixeelCam2 = matrix_from_file('cam0pixelsIdeal.txt')
    points3D=matrix_from_file('points3D.txt')





def matrix_from_file(file_name):
    with open(file_name, 'r') as f:
        l = [[np.float64(num) for num in line.split(',')] for line in f]
    return np.array(l,dtype=np.float64)













if __name__=="__main__":
    main()