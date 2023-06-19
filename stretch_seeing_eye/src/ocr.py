import rospy
import cv2
import numpy as np
import pytesseract
import cv_bridge
import re

from sensor_msgs.msg import Image


def auto_canny_edge_detection(image, sigma=0.33):
    md = np.median(image)
    lower_value = int(max(0, (1.0-sigma) * md))
    upper_value = int(min(255, (1.0+sigma) * md))
    return cv2.Canny(image, lower_value, upper_value)


def callback(msg: Image):
    bridge = cv_bridge.CvBridge()
    imageRGB = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    imageRGB = cv2.resize(imageRGB, (0, 0), fx=0.25, fy=0.25)
    gray = cv2.cvtColor(imageRGB, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    auto_edge = auto_canny_edge_detection(blurred)
    # cv2.imshow('auto_edge', auto_edge)
    # cv2.waitKey(0)

    kernel = np.ones((2, 2), np.uint8)
    closing = cv2.morphologyEx(auto_edge, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(
        closing.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    hierarchy = hierarchy[0]

    temp1 = np.zeros_like(closing)
    temp2 = np.zeros_like(closing)

    for i, cnt in enumerate(contours):
        if hierarchy[i][2] > -1 and hierarchy[i][3] < 0:
            cv2.drawContours(temp1, contours, i, 255, -1)
            t = hierarchy[i][2]
            if cv2.contourArea(contours[i]) - cv2.contourArea(contours[t]) > 50:
                cv2.drawContours(temp2, contours, t, 255, -1)
            cv2.contourArea(contours[t])
            t = hierarchy[t][2]
            while t > -1 and t < len(contours):
                cv2.drawContours(temp2, contours, t, 255, -1)
                t = hierarchy[t][2]

    temp3 = temp1 - temp2

    rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 3))
    dilation = cv2.dilate(temp3, rect_kernel, iterations=1)
    contours, hierarchy = cv2.findContours(
        dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hierarchy = hierarchy[0]

    for i, cnt in enumerate(contours):
        x, y, w, h = cv2.boundingRect(cnt)
        crop = imageRGB[y-10:y+h+10, x-10:x+w+10]
        final = crop
        words: str = pytesseract.image_to_string(
            final, lang='eng', config=r'--oem 3 --psm 6')
        words = words.strip()
        if len(words) > 0 and re.match(r'[A-Z0-9]+', words) is not None:
            print(words)


def main():
    rospy.init_node('ocr')
    rospy.Subscriber('/static_image', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
