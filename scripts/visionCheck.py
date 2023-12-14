#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import numpy as np
import cv2
import math

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('rs_check_assembly', String, queue_size=10)
pubObj = rospy.Publisher('rs_object_detection', Bool, queue_size=1)

# Message variable for the color code to publish.
colorCodeDetected = String()

# List variable to define the color code of the product.
yMeasured = []

# Variable to save the camera view at the start, useful for object detection.
background = None

#--------- FUNCTION ---------#
# Function to perform object detection operation by checking if the product
# is in a specified area of the image.
def objectDetection(data):
    # Capture the image.
    webcam = cv2.VideoCapture(2)
    rospy.sleep(0.3) # Wait to capture the image when the camera is ready.
    result, image = webcam.read()
    webcam.release()

    if result:
        # Subtract the background to isolate the product.
        subtractedFrame = cv2.absdiff(background, image)

        # Convert the subtracted image to grayscale.
        grayFrame = cv2.cvtColor(subtractedFrame, cv2.COLOR_BGR2GRAY)

        # Apply thresholding to highlight the product.
        _, thresholdedMask = cv2.threshold(grayFrame, 60, 255, cv2.THRESH_BINARY)

        # Create a contour around the detected product area.
        xObject = createContour(image, thresholdedMask, (0, 0, 255), "Object", 500, "x")
        
        # Get the witdh of the image frame.
        widthFrame = image.shape[1]

        # Check if the product is in the left half of the image.
        if data.data == "L":
            # Publish True if the check is successful; otherwise, publish False.
            if xObject < math.floor(widthFrame/2) and xObject != -1:
                pubObj.publish(True)
            else:
                pubObj.publish(False)

        # Check if the product is in the right half of the image.
        elif data.data == "R":
            # Publish True if the check is successful; otherwise, publish False.
            if xObject > math.floor(widthFrame/2):
                pubObj.publish(True)
            else:
                pubObj.publish(False)
        
# Function to perform the check assembly operation,
# determining the color code of the product.
def checkAssembly(data):
    # Capture the image.
    webcam = cv2.VideoCapture(2)
    rospy.sleep(0.3) # Wait to capture the image when the camera is ready.
    result, imageFrame = webcam.read()
    webcam.release()

    # Reset the yMeasured value
    yMeasured = []

    # Define the colors to check.
    colorsToCheck = ["R", "G", "B"]

    if result:
        # Convert the image to HSV (Hue Saturation Brightness).
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Note: The range values may depend on the camera used and other parameters,
        # so it is advisable to perform a calibration to determine optimal thresholding values.
        # Set range values for red color.
        red_lower = np.array([0, 217, 90], np.uint8)
        red_upper = np.array([179, 255, 134], np.uint8)

        # Set range values for green color.
        green_lower = np.array([53, 91, 23], np.uint8)
        green_upper = np.array([96,255,255], np.uint8)

        # Set range values for blue color.
        blue_lower = np.array([106, 163, 23], np.uint8)
        blue_upper = np.array([146, 255, 255], np.uint8)

        # Define a kernel of size 5x5, later used for dilation.
        kernel = np.ones((5, 5), "uint8")

        for color in colorsToCheck:
            if color == "R": 
                # Find the elements with a red color.
                red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
                red_mask = cv2.dilate(red_mask, kernel)
                res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                        mask = red_mask)
                
                # Create a contours to track red color, returning the y of the
                # contour with the largest area.
                yR = createContour(imageFrame, red_mask, (0,0,255), "Red", 300, "y")

                # Add the y value of the red element detected.
                yMeasured.append(yR)

            elif color == "G":
                # Find the elements with a green color.
                green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
                green_mask = cv2.dilate(green_mask, kernel)
                res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                            mask = green_mask)
                
                # Create a contours to track green color, returning the y of the 
                # contour with the largest area.
                yG = createContour(imageFrame, green_mask, (0,255,0), "Green", 300, "y")

                # Add the value of the y of the green element detected.
                yMeasured.append(yG)

            elif color == "B":
                # Find the elements with a blue color.
                blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
                blue_mask = cv2.dilate(blue_mask, kernel)
                res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                                        mask = blue_mask)
                
                # Create a contours to track blue color, returning the y of the 
                # contour with the largest area.
                yB = createContour(imageFrame, blue_mask, (255,0,0), "Blue", 300, "y")

                # Add the y value of the blue element detected.
                yMeasured.append(yB)
        
        # Determine the color code detected from the y measured for each color element,
        # and subsequently publish it.
        colorCodeDetected.data = setColorCodeDetected(yMeasured, colorsToCheck)
        pub.publish(colorCodeDetected)
        
# Function that creates contours in the image frame based on the specified mask.
# Note: This function is used in both object detection and check assembly operations.
# The axis to be measured for the contour to be returned is different between the two
# operations, so the axis needs to be specified.
def createContour(imageFrame, mask, bgr_color:tuple, text: str, minArea: int, axisToMeasure: str):
    # Find the countours in the mask.
    contours, hierarchy = cv2.findContours(mask,
                                        cv2.RETR_TREE,
                                        cv2.CHAIN_APPROX_SIMPLE)
    
    # Variables used to get axis value to return.
    maxArea = 0
    measureMax = -1
    
    # Create for each countour detected a rectangle. Also, find the element
    # with the biggest area and return its value of the axis that needs to be mesured.
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        # Note: The minArea is threshold value below which the found element is not considered.
        if(area > minArea):
            x, y, w, h = cv2.boundingRect(contour)
            if area > maxArea:
                if axisToMeasure == "y":
                    measureMax = y
                elif axisToMeasure == "x":
                    measureMax = x
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                    (x + w, y + h),
                                    bgr_color, 2)
            
            cv2.putText(imageFrame, text, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, bgr_color)
            
    return measureMax

# Function that returns the color code detected.
# Note: The origin of the image in the top left corner with y-axis direction downwards.
def setColorCodeDetected(yMeasured, colors):
    # Link the list of yMesured and colors. They are both with the RGB order.
    combinedList = list(zip(yMeasured, colors))
    # Sort the list in descending order.
    combinedList.sort(reverse=True)
    # Divide the lists.
    ySorted, colorCodeDetected = zip(*combinedList)

    # Exclude colors from the color code that weren't detected (y value is -1 in those cases).
    colorCodeDetected = [color for i, color in enumerate(colorCodeDetected) if ySorted[i] != -1]
    
    # String that will be publish if no color is detected.
    stringToPublish = "No color found"

    # Add the color in order to the string to publish.
    for color in colorCodeDetected:
        if stringToPublish == "No color found":
            stringToPublish = "%s" % color
        else:
            stringToPublish = stringToPublish + ";%s" % color

    return stringToPublish

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node.                
def listener():
    rospy.init_node('vision_operation', anonymous=True)
    rospy.Subscriber("check_assembly", String, checkAssembly)
    rospy.Subscriber("object_detection", String, objectDetection)
    rospy.spin()

if __name__ == "__main__":
    # Get the camera view a the start. This will be useful in the object detection to
    # subtract the background to isolate the product.
    webcam = cv2.VideoCapture(2)
    rospy.sleep(0.3)
    resultBG, background = webcam.read()
    webcam.release()

    # Initialize the ROS node and start the subscribers.
    if resultBG:
        listener()