import cv2
import numpy as np

# Load the image (or capture from a camera)
image = cv2.imread('object.jpg')

# Convert the image to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Manually select a region of interest (ROI) using OpenCV's selectROI function
# The ROI should be a region where the object color is most prominent
roi = cv2.selectROI('Select ROI', image, showCrosshair=True)
roi_image = hsv_image[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]

# Calculate the minimum and maximum HSV values within the selected ROI
hsv_min = np.min(roi_image, axis=(0, 1))
hsv_max = np.max(roi_image, axis=(0, 1))

# Print the HSV boundaries
print(f"HSV Min: {hsv_min}")
print(f"HSV Max: {hsv_max}")

# To visualize the selected ROI and boundaries, you can create a mask
mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
result = cv2.bitwise_and(image, image, mask=mask)

# Display the images
cv2.imshow('Selected ROI', image[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])])
cv2.imshow('Mask', mask)
cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()