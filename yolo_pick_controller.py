# yolo_pick_controller.py
from controller import Robot
from ultralytics import YOLO
import cv2
import numpy as np
import time

# -------------------------------
# Robot setup
# -------------------------------
robot = Robot()
TIME_STEP = 64

# Wheels
LEFT_WHEEL = "wheel_left_joint"
RIGHT_WHEEL = "wheel_right_joint"
left_motor = robot.getDevice(LEFT_WHEEL)
right_motor = robot.getDevice(RIGHT_WHEEL)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Arm joints
ARM_JOINTS = [
    "arm_1_joint", "arm_2_joint", "arm_3_joint", 
    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
]
arm_motors = [robot.getDevice(name) for name in ARM_JOINTS]

# Gripper
GRIPPER_LEFT = "gripper_left_finger_joint"
GRIPPER_RIGHT = "gripper_right_finger_joint"
gripper_left = robot.getDevice(GRIPPER_LEFT)
gripper_right = robot.getDevice(GRIPPER_RIGHT)

# Camera
CAMERA_NAME = "camera"
camera = robot.getDevice(CAMERA_NAME)
camera.enable(TIME_STEP)
width = camera.getWidth()
height = camera.getHeight()

# -------------------------------
# YOLO setup
# -------------------------------
MODEL_PATH = r"C:\MY\UH\Apllied robotic\project_library\worlds\New folder\controllers\photo\best.pt"
yolo_model = YOLO(MODEL_PATH)

# -------------------------------
# Helper functions
# -------------------------------
def capture_frame():
    """Capture current frame from camera as a numpy array"""
    img = np.frombuffer(camera.getImage(), np.uint8).reshape((height, width, 4))
    return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

def move_arm_to_pick():
    """Simple arm pick simulation"""
    print("Moving arm to pick position...")
    for i, motor in enumerate(arm_motors):
        motor.setPosition(0.5)  # adjust per joint
    time.sleep(1.0)
    print("Closing gripper...")
    gripper_left.setPosition(0.03)
    gripper_right.setPosition(0.03)
    time.sleep(0.5)
    print("Arm pick simulation complete.")
    # Open gripper after pick
    gripper_left.setPosition(0.0)
    gripper_right.setPosition(0.0)
    time.sleep(0.5)

def move_robot_toward_book(x1, y1, x2, y2):
    """Move TIAGo toward the book based on bounding box center"""
    img_center_x = width // 2
    book_center_x = int((x1 + x2) / 2)
    offset = book_center_x - img_center_x

    # Rotate to align
    if abs(offset) > 20:
        if offset > 0:
            left_motor.setVelocity(0.3)
            right_motor.setVelocity(-0.3)
            print("Rotating right to align with book...")
        else:
            left_motor.setVelocity(-0.3)
            right_motor.setVelocity(0.3)
            print("Rotating left to align with book...")
        return False
    else:
        # Move forward
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
        print("Moving forward toward book...")
        return True

# -------------------------------
# Main loop
# -------------------------------
print("Starting YOLO pick controller...")
picked_books = set()  # track picked books

while robot.step(TIME_STEP) != -1:
    frame = capture_frame()
    results = yolo_model.predict(source=frame, save=False, verbose=False)

    detected = False
    for r in results:
        boxes = r.boxes
        if len(boxes) > 0:
            detected = True
            for idx, box in enumerate(boxes):
                # Skip already picked books
                if idx in picked_books:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                print(f"Detected book {idx+1} at ({x1},{y1},{x2},{y2})")
                
                # Move robot toward book until aligned
                aligned = move_robot_toward_book(x1, y1, x2, y2)
                if aligned:
                    # Stop wheels
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    move_arm_to_pick()
                    picked_books.add(idx)
                    time.sleep(1.0)

    # Show camera frame
    cv2.imshow("TIAGo Camera YOLO", frame)
    cv2.waitKey(1)

    # If all books picked, exit
    if detected and len(picked_books) == len(boxes):
        print("All detected books picked.")
        break

print("Controller finished.")
cv2.destroyAllWindows()
