import cv2  
import numpy as np
import time

# Trackbar callback function to update HSV value.
def callback(x):
	global H_low,H_high,S_low,S_high,V_low,V_high

	# Assign trackbar position value to H,S,V High and low variable.
	H_low = cv2.getTrackbarPos('low H','controls')
	H_high = cv2.getTrackbarPos('high H','controls')
	S_low = cv2.getTrackbarPos('low S','controls')
	S_high = cv2.getTrackbarPos('high S','controls')
	V_low = cv2.getTrackbarPos('low V','controls')
	V_high = cv2.getTrackbarPos('high V','controls')

# Create a seperate window named 'controls' for trackbars.
cv2.namedWindow('controls',2)
cv2.resizeWindow("controls", 550,510)

# Global variables.
H_low = 0
H_high = 179
S_low= 0
S_high = 255
V_low= 0
V_high = 255

# Get the image.
webcam = cv2.VideoCapture(2)
time.sleep(0.4)
result, img = webcam.read()

# Create trackbars for high,low H,S,V. 
cv2.createTrackbar('low H','controls',0,179,callback)
cv2.createTrackbar('high H','controls',179,179,callback)

cv2.createTrackbar('low S','controls',0,255,callback)
cv2.createTrackbar('high S','controls',255,255,callback)

cv2.createTrackbar('low V','controls',0,255,callback)
cv2.createTrackbar('high V','controls',255,255,callback)

while(result):
	# Convert the image to HSC color mode.
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Define the HSV range.
	hsv_low = np.array([H_low, S_low, V_low], np.uint8)
	hsv_high = np.array([H_high, S_high, V_high], np.uint8)

	img_not = cv2.bitwise_not(img)

	# Making mask for hsv range.
	mask = cv2.inRange(hsv, hsv_low, hsv_high)
	print (mask)

	# Masking HSV value selected color becomes black
	res = cv2.bitwise_and(img, img, mask=mask)

	# Show image
	cv2.imshow('not', img_not)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	
	# Wait for the user to press escape and break the while loop.
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break
		
# Destroys all window.
cv2.destroyAllWindows()
