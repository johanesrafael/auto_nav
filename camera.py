
#Color Identification
import cv2
import numpy as np
#import imutils

cap = cv2.VideoCapture(0)

while True:
	_, frame = cap.read()
#	print(frame)
	hsv_frame = cv2.cvtColor(np.uint8(frame), cv2.COLOR_BGR2HSV)
	

	#Red color
	low_red = np.array([166, 155, 84])
	high_red = np.array([179, 255, 255])
	red_mask = cv2.inRange(hsv_frame, low_red, high_red)
	red = cv2.bitwise_and(frame, frame, mask=red_mask)
	
	#Blue color
	low_blue = np.array([90, 80, 2])
	high_blue = np.array([128, 255, 255])
        blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
        blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

	#Green color
	low_green = np.array([27, 50, 72])
	high_green = np.array([105, 255, 255])
        green_mask = cv2.inRange(hsv_frame, low_green, high_green)
        green = cv2.bitwise_and(frame, frame, mask=green_mask)
	
	#Purple color
	#Cyan color
	#Magenta color
	#Yellow color
	#White color
	#Black color

	#Only identify the specific color, the rest is ignored
#	(regions, _) = hog.detectMultiScale(frame, 
 #                                           winStride=(4, 4), 
  #                                          padding=(4, 4), 
   #                                         scale=1.05) 
#	for(x,y) in regions:
#		cv2.rectangle(frame , (x,y), (0,0,255), 2)


	cv2.imshow("Frame", frame)
	cv2.imshow("Red", red)
#	cv2.imshow("Blue", blue)
#	cv2.imshow("Green", green)

	print(red)

	key = cv2.waitKey(1)
	if key == 27:
		break



#Every color needs tuning, trial and error are needed or color sample
