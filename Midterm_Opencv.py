# The same libraries used in Lab 9 are used in this open_cv version of code
import numpy as np
import cv2 as cv
import time
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
from Motor import *            
PWM = Motor()

# The methodology is simple. Control the robot to make it at the same side of obstacle as the duck
# Use opencv to capture duck's position and send feedback to the Robo
# Robo will follow the duck like follow the ball of color
# The following code will drive the robot to make it at the same side of obstacle as the duck
# We use the time.sleep function to control how sharp the turn is, and how far will the robot go straightforward
# All the parameters for motors are obtained by trails and fails

# Go straight
PWM.setMotorModel(1000, 1000, 1000, 1000)
time.sleep(1.1)
# Make a turn
PWM.setMotorModel(2000, 2000, -2000, -2000)
time.sleep(0.65)
# Go straight
PWM.setMotorModel(1000, 1000, 1000, 1000)
time.sleep(1.9)
# Make a turn
PWM.setMotorModel(2000, 2000, -2000, -2000)
time.sleep(0.65)
# When the robot and the duck at the same side of the obstacle we close the motors
PWM.setMotorModel(0, 0, 0, 0)
# The main loop is here. The usage is to guide the robot with the position of duck

try:  
    while True:
        # This two lines of code will be used to capture camera vision
        # Copies from template code
        frame = picam2.capture_array()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # lower and upper boundaries of yellow color in HSV color space
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        # https://www.geeksforgeeks.org/filter-color-with-opencv/
        # This code comes from the above websites. The usage is color mask
        mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv.bitwise_and(frame, frame, mask=mask)
        # The following code is a part of the hough transform
        # https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        # The key part is here: using opencv to get duck position and guide the robot
        # calculate moments of a binary image
        MOMENTS = cv.moments(gray)
        # calculate the X and Y coordinates of the object center
        # The websites contains information about "m00", "m10", etc...
        if MOMENTS["m00"] == 0:
            PWM.setMotorModel(-1000, -1000, 1000, 1000)
            time.sleep(0.04)
            PWM.setMotorModel(0, 0, 0, 0)
            time.sleep(0.02)
        else:
            # Thresholds used to determine where the robo should head
            Coordinate_Threshold_Left = 290
            Coordinate_Threshold_Right = 310
            # The following code used this reference: https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
            Coordinate_X = int(MOMENTS["m10"] / MOMENTS["m00"])
            Coordinate_Y = int(MOMENTS["m01"] / MOMENTS["m00"])
            # Show the center position of the object
            cv.circle(frame, (Coordinate_X, Coordinate_Y), 5, (255, 255, 255), -1)
            # Put a text at the center
            cv.putText(frame, "center", (Coordinate_X - 25, Coordinate_Y - 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # Define a very simple control policy here:
            # If the robot is heading somewhere on the left of duck, turn right
            # If the robot is heading somewhere on the right of duck, turn left
            # The X coordinate of the center is roughly 300
            if Coordinate_X < Coordinate_Threshold_Left:
                PWM.setMotorModel(-1000, -1000, 1000, 1000)
                time.sleep(0.02)
                PWM.setMotorModel(0, 0, 0, 0)
            elif Coordinate_X > Coordinate_Threshold_Right:
                PWM.setMotorModel(1000, 1000, -1000, -1000)
                time.sleep(0.02)
                PWM.setMotorModel(0, 0, 0, 0)
            else:
                break
        # Break if loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
# The robot should have captured the duck
# If there is keyboard interrupt then break
except KeyboardInterrupt:
    PWM.setMotorModel(0, 0, 0, 0)
    cv.destroyAllWindows()
# The following code will drive the robot go back to the starting position
# Go straight
PWM.setMotorModel(6000, 6000, 6000, 6000)
time.sleep(0.7)
PWM.setMotorModel(2500, 2500, 2500, 2500)
time.sleep(0.5)
# Make a turn
PWM.setMotorModel(2000, 2000, -2000, -2000)
time.sleep(0.70)
# Go straight
PWM.setMotorModel(1000, 1000, 1000, 1000)
time.sleep(2)
# Make a turn
PWM.setMotorModel(2000, 2000, -2000, -2000)
time.sleep(0.67)
# Go straight
PWM.setMotorModel(1000, 1000, 1000, 1000)
time.sleep(1.2)

# This is used to close the motors after the program is executed
PWM.setMotorModel(0, 0, 0, 0)
cv.destroyAllWindows()
print("Execution ends Here")
