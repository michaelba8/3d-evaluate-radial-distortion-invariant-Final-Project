import numpy as np
import cv2
from wand.image import Image
import os

def main():
    path="projection results\iron man/image5.png"

    img = cv2.imread(path)
    coef=(0.15,0,0,1)
    newimg=apply_distortion_lib(path,coef)

    # Display old and new image
    cv2.imshow('ideal',img)
    cv2.imshow('distorted',newimg)
    cv2.waitKey()





def apply_distortion_lib(path,coef=(0.5, 0.0, 0.0, 1.0)):
    with Image(filename=path) as img:
        print(img.size)
        #img.virtual_pixel = 'transparent'
        img.distort('barrel', coef)
        img_opencv = np.array(img)

    return img_opencv


def read_images(folder):
    """Reading all the image from folder"""
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(filename,0)
        if img is not None:
            images.append(img)

    return images


if __name__=='__main__':
    main()