#!/usr/bin/env python3

import cv2

cap = cv2.VideoCapture("Lane_test_video.mp4")

while cap.isOpened():
    ret,frame = cap.read()
    if not ret:
        print("No frame")
        break
    
    cv2.imshow('video', frame)
    
    if cv2.waitKey(1) == ord('q'):
        print('end by the user')
        break
    
cap.release()
cv2.destroyAllWindows()
