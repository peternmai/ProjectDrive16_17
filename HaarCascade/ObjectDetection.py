# Title: Object Detection
# Author: Peter Mai and Harrison Kinsley
# Reference: https://pythonprogramming.net/haar-cascade-object-detection-python-opencv-tutorial/
#
# This program will read in the haar cascade xml file. Then it'll open a camera
# and try to use it to track the object of interest.

import os
import sys
import cv2

# Ask for haar cascade file and specification
haarcascade_file = input("Type name of haar cascade xml file: ")
scale_factor = float(input("Type scale factor (default: 1.3) [DECIMAL]:  "))
min_neighbor = int(input("Type accuracy level (higher = more accurate) [INTEGER]: "))

# Try to open haar cascade trained file
if os.path.isfile('./trained/' + haarcascade_file):
    water_bottle_cascade = cv2.CascadeClassifier('trained/' + haarcascade_file)
else:
    print("Error. Invalid file: " + os.path.dirname(os.path.abspath(__file__)) + '/trained/' + haarcascade_file)
    sys.exit()

# Get video feed from default camera (webcam)
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 2nd parameter = scale factor, 3rd = minNeighbor, how sure?
    water_bottle = water_bottle_cascade.detectMultiScale(gray, scale_factor, min_neighbor)
    for (x,y,w,h) in water_bottle:
        cv2.rectangle(img, (x,y), (x+w, y+h), (225,225,0), 2)

    # Show image
    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
