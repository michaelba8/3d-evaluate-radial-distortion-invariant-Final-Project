import numpy as np
import cv2
from wand.image import Image
import os

def main():
    to_save=True
    image_name='image3distorted.png'
    folder_result = "projection results\iron man"
    cur_dir=os.path.dirname(__file__)
    path="projection results\iron man/image3.png"

    img = cv2.imread(path)
    coef=(0.2,0,0,1)
    newimg=apply_distortion(path,coef)

    # Display old and new image
    cv2.imshow('ideal',img)
    cv2.imshow('distorted',newimg)
    cv2.waitKey()
    os.chdir(os.path.join(cur_dir, folder_result))
    if (to_save):
        cv2.imwrite(image_name, newimg)





def apply_distortion(path,coef=(0.8, 0.0, 0.0, 1.0)):
    with Image(filename=path) as img:
        print(img.size)
        img.virtual_pixel = 'transparent'
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