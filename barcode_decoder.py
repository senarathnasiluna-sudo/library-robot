from controller import Robot, Camera
import cv2
import numpy as np
from pyzbar.pyzbar import decode

# Initialize robot and camera
robot = Robot()
camera = robot.getDevice("Astra rgb")
camera.enable(64)

# Main loop
while robot.step(64) != -1:
    # Get image from camera
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    # Convert Webots image → OpenCV format
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
    
    # --- Decode barcodes in the image ---
    barcodes = decode(img_bgr)
    
    for barcode in barcodes:
        # Extract bounding box and data
        (x, y, w, h) = barcode.rect
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type
        
        # Draw rectangle and label
        cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = f"{barcode_data} ({barcode_type})"
        cv2.putText(img_bgr, text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        print("Detected:", barcode_data, "| Type:", barcode_type)
    
    # Show camera feed
    cv2.imshow("Tiago Camera", img_bgr)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()