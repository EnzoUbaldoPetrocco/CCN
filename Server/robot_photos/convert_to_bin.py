import cv2
import base64

# Read the image
img = cv2.imread('535857830000.png')  # Replace with your actual filename

# Encode image as PNG in memory
success, buffer = cv2.imencode('.png', img)
if not success:
    raise Exception("Could not encode image")

# Base64 encode the PNG bytes
img_base64 = base64.b64encode(buffer)

# Save base64 bytes to a .bin file
with open('output_image_base64.bin', 'wb') as f:
    f.write(img_base64)