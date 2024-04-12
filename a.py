from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
def read_and_preproces(filename, canny_low= 2, canny_high = 255, blur_kernel=3, d_e_kernel=125):
    # Đọc file ảnh
    image = cv2.imread(filename)
    # Chuyển thành ảnh xám
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Làm mờ ảnh

    v = np.median(image)
    gray = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 1)


    # Áp dụng Canny tìm cạnh
    edged = cv2.Canny(gray, canny_low, canny_high)
    edged = cv2.dilate(edged, (d_e_kernel, d_e_kernel), iterations=1)
    edged = cv2.erode(edged, (d_e_kernel, d_e_kernel), iterations=1)
    return image, edged

image, edged = read_and_preproces('a.JPG')
cv2.imshow("B", edged)
cv2.waitKey()
