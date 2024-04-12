from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2

ref_width = 20  # in mm
v_xu= 0.3 # cm3
lst1=[]
lst2=[]

def read_and_preproces(filename, canny_low= 2, canny_high = 255, blur_kernel=5, d_e_kernel=120):
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

image1, edged1 = read_and_preproces('oitx.jpg')
cv2.imshow("B", edged1)
cv2.waitKey()
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) / 2, (ptA[1] + ptB[1]) / 2)


def get_distance_in_pixels(orig, c):
    # Lấy minRect
    box = cv2.minAreaRect(c)
    # Lấy tọa độ các đỉnh của MinRect
    box = cv2.boxPoints(box)
    box = np.array(box, dtype="int")
    # Sắp xếp các điểm theo trình tự
    box = perspective.order_points(box)
    # Vẽ contour
    cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
    # Tinh toán 4 trung diểm của các cạnh
    (tl, tr, br, bl) = box
    (tltrX, tltrY) = midpoint(tl, tr)
    (blbrX, blbrY) = midpoint(bl, br)
    (tlblX, tlblY) = midpoint(tl, bl)
    (trbrX, trbrY) = midpoint(tr, br)
    # Tính độ dài 2 chiều
    dc_W = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
    dc_H = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

    return dc_W, dc_H, tltrX, tltrY, trbrX, trbrY


def find_object_in_pix(orig, edged1, area_threshold=1000):
    # Tìm các Contour trong ảnh
    cnts = cv2.findContours(edged1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # Sắp xếp các contour từ trái qua phải
    (cnts, _) = contours.sort_contours(cnts)
    P = None

    # Duyệt các contour
    for c in cnts:
        # Nếu contour quá nhỏ -> bỏ qua
        if cv2.contourArea(c) < area_threshold:
            continue

        # Tính toán 2 chiều bằng Pixel
        dc_W, dc_H, tltrX, tltrY, trbrX, trbrY = get_distance_in_pixels(orig, c)

        # Nếu là đồng xu
        if P is None:
            # Cập nhật số P
            P = ref_width / dc_H
            # Gán luôn kích thước thật bằng số đã biết
            dr_W = ref_width
            dr_H = ref_width
        else: # Nếu là các vật khác
            # Tính toán kích thước thật dựa vào kích thước pixel và số P
            dr_W = dc_W * P
            dr_H = dc_H * P
            lst1.append(dr_H)
            lst1.append(dr_W)
        print(dr_H)
        print(dr_W)
        # Ve kich thuoc len hinh
        cv2.putText(orig, "{:.1f} mm".format(dr_H), (int(tltrX - 15), int(tltrY - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
        cv2.putText(orig, "{:.1f} mm".format(dr_W), (int(trbrX + 10), int(trbrY)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
        print("Dien tich vat the ", )

    return orig

def find_object_in_pix2(orig, edged2, area_threshold=1000):
    # Tìm các Contour trong ảnh
    cnts = cv2.findContours(edged2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # Sắp xếp các contour từ trái qua phải
    (cnts, _) = contours.sort_contours(cnts)
    P = None

    # Duyệt các contour
    for c in cnts:
        # Nếu contour quá nhỏ -> bỏ qua
        if cv2.contourArea(c) < area_threshold:
            continue

        # Tính toán 2 chiều bằng Pixel
        dc_W, dc_H, tltrX, tltrY, trbrX, trbrY = get_distance_in_pixels(orig, c)

        # Nếu là đồng xu
        if P is None:
            # Cập nhật số P
            P = ref_width / dc_H
            # Gán luôn kích thước thật bằng số đã biết
            dr_W = ref_width
            dr_H = ref_width
        else: # Nếu là các vật khác
            # Tính toán kích thước thật dựa vào kích thước pixel và số P
            dr_W = dc_W * P
            dr_H = dc_H * P
            lst2.append(dr_H)
            lst2.append(dr_W)
        print(dr_H)
        print(dr_W)
        # Ve kich thuoc len hinh
        cv2.putText(orig, "{:.1f} mm".format(dr_H), (int(tltrX - 15), int(tltrY - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
        cv2.putText(orig, "{:.1f} mm".format(dr_W), (int(trbrX + 10), int(trbrY)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        print("Dien tich vat the ", )
    return orig


image1 = find_object_in_pix(image1, edged1)
cv2.imshow("A", image1)
cv2.waitKey()
cv2.destroyAllWindows()

image2, edged2 = read_and_preproces('oitp.jpg')
cv2.imshow("B", edged2)
cv2.waitKey()

image2 = find_object_in_pix2(image2, edged2)
cv2.imshow("A", image2)
cv2.waitKey()
cv2.destroyAllWindows()
print(lst1)
print(lst2)
v=round(float(lst1[0]),2)*round(float(lst1[1]),2)*round(float(lst2[1]),2)
v=round(v/1000,3)
print("Thể tích vật thể:",v)

def thetich(orig, edged1,v, area_threshold=1000):
    # Tìm các Contour trong ảnh
    cnts = cv2.findContours(edged1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # Sắp xếp các contour từ trái qua phải
    (cnts, _) = contours.sort_contours(cnts)
    P = None

    # Duyệt các contour
    for c in cnts:
        # Nếu contour quá nhỏ -> bỏ qua
        if cv2.contourArea(c) < area_threshold:
            continue

        # Tính toán 2 chiều bằng Pixel
        dc_W, dc_H, tltrX, tltrY, trbrX, trbrY = get_distance_in_pixels(orig, c)

        # Nếu là đồng xu
        if P is None:
            # Cập nhật số P
            P = ref_width / dc_H
            cv2.putText(orig, "{:.1f} cm^3".format(v_xu), (int(tltrX - 15), int(tltrY - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
        else: # Nếu là các vật khác
            cv2.putText(orig, "{:.1f} cm^3".format(v), (int(tltrX - 15), int(tltrY - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
    return orig
image3, edged3 = read_and_preproces('oitp.jpg')
image3 = thetich(image3, edged3,v)
cv2.imshow("A", image3)
cv2.waitKey()
cv2.detroyAllWindows()