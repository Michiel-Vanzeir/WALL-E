import numpy as np
import cv2

# Initialize videofeed the parameters
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()
    
    # Do some processing on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 25, 255, cv2.THRESH_BINARY_INV)

    # Find biggest contour and draw it
    contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 3)

    # Find the center of the biggest contour
    try:
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    except:
        pass

    # Draw the center of the biggest contour
    cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)

    # Decide which way to turn
    if cx < 120:
        print("Turn left")
    elif cx > 320:  
        print("Turn right")
    else:
        print("Go straight")

    cv2.imshow("frame", frame)


    if cv2.waitKey(333) & 0xFF == ord('q'):
        break


