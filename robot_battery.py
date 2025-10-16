from controller import Robot, Camera, Motor, DistanceSensor, Gyro, Lidar, Accelerometer
import numpy as np
import cv2
import random
import math

# ---------------- CONFIG ----------------
TIMESTEP = 64
CAMERA_NAME = "Astra rgb"
SONARS = ["base_sonar_01_link", "base_sonar_02_link", "base_sonar_03_link"]
GYRO_NAME = "gyro"
LIDAR_NAME = "Hokuyo URG-04LX-UG01"
ACCEL_NAME = "accelerometer"
LEFT_MOTOR_NAME = "wheel_left_joint"
RIGHT_MOTOR_NAME = "wheel_right_joint"

BATTERY_THRESHOLD = 20
BATTERY_MAX = 100
BATTERY_DRAIN_PER_STEP = 0.2
CHARGE_STEP = 5
BLACK_PIXEL_COUNT_THRESHOLD = 1800
SEARCH_TIMEOUT = 10.0

# ---------------- INIT ----------------
robot = Robot()
battery_level = 100.0
charging = False
current_task = "color_detection"
last_task = None

# ---------------- DEVICES ----------------
# Camera
camera = robot.getDevice(CAMERA_NAME)
camera.enable(TIMESTEP)
cam_width = camera.getWidth()
cam_height = camera.getHeight()
print(f"[INFO] Camera '{CAMERA_NAME}' enabled ({cam_width}x{cam_height})")

# Motors
left_motor = robot.getDevice(LEFT_MOTOR_NAME)
right_motor = robot.getDevice(RIGHT_MOTOR_NAME)
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
print(f"[INFO] Wheel motors configured.")

# Distance Sensors
sonars = []
for name in SONARS:
    sensor = robot.getDevice(name)
    sensor.enable(TIMESTEP)
    sonars.append(sensor)
print("[INFO] Sonar sensors enabled.")

# Gyro
gyro = robot.getDevice(GYRO_NAME)
gyro.enable(TIMESTEP)
print("[INFO] Gyro enabled.")

# Lidar
lidar = robot.getDevice(LIDAR_NAME)
lidar.enable(TIMESTEP)
lidar.enablePointCloud()
print("[INFO] Lidar enabled.")

# Accelerometer
try:
    accel = robot.getDevice(ACCEL_NAME)
    accel.enable(TIMESTEP)
    print("[INFO] Accelerometer enabled.")
except:
    accel = None
    print("[WARN] Accelerometer not found.")

# ---------------- MOTOR CONTROL ----------------
def stop_robot():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def move_forward(speed=3.0):
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)

def rotate_in_place(speed=2.0, direction="left"):
    if direction == "left":
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
    else:
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)

# ---------------- CAMERA PROCESSING ----------------
def webots_image_to_bgr(image):
    if image is None:
        return None
    img = np.frombuffer(image, dtype=np.uint8)
    channels = 4 if len(img) == cam_width * cam_height * 4 else 3
    img = img.reshape((cam_height, cam_width, channels))
    return img[:, :, :3].copy()

def detect_black_in_frame(bgr):
    if bgr is None:
        return None, 0, (0, 0)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    black_pixels = cv2.countNonZero(mask)
    cx, cy = 0, 0
    if black_pixels > 0:
        moments = cv2.moments(mask)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
    return mask, black_pixels, (cx, cy)

def obstacle_detected():
    readings = [s.getValue() for s in sonars]
    front_left, front, front_right = readings
    if front < 800 or front_left < 800 or front_right < 800:
        return True, readings
    return False, readings

# ---------------- OBSTACLE AVOIDANCE ----------------
def obstacle_avoidance_task():
    obstacle, sonar_vals = obstacle_detected()
    if obstacle:
        print(" [TASK] Avoiding obstacle...")
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(2.0)
    else:
        move_forward(3.0)

# ---------------- BLACK SEARCH + CHARGING ----------------
def search_and_move_to_black(timeout=SEARCH_TIMEOUT):
   
    print(" Searching for black charging station...")
    start_time = robot.getTime()
    did_circle = False  # ensures one full circle happens only once per search
    while robot.step(TIMESTEP) != -1:
        image = camera.getImage()
        bgr = webots_image_to_bgr(image)
        mask, black_pixels, (cx, cy) = detect_black_in_frame(bgr)
        obstacle, sonar_vals = obstacle_detected()

        # --- OpenCV visualization ---
        if bgr is not None:
            if black_pixels > 0:
                cv2.circle(bgr, (cx, cy), 10, (0, 0, 255), 2)
            combined = np.hstack((bgr, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
            cv2.imshow("Camera (RGB + Black Mask)", combined)
            cv2.waitKey(1)

        # Accelerometer shock detection
        if accel:
            ax, ay, az = accel.getValues()
            if abs(ax) > 10 or abs(ay) > 10 or abs(az - 9.8) > 5:
                print(" Shock detected! Stopping robot.")
                stop_robot()
                continue

        if obstacle:
            print(" Obstacle detected! Rotating to avoid.")
            rotate_in_place(2.0, "left")
            robot.step(TIMESTEP)
            continue

        # If we detect a significant black area, circle once first (to scan) then approach
        if black_pixels > BLACK_PIXEL_COUNT_THRESHOLD and not did_circle:
            # Announce detection and circling behavior
            print(" Black area detected! Circling once to locate charging station...")
            print(" Circling now (360°) to confirm exact location...")
            # Perform a timed circle: rotate in place for a short duration to approximate one full turn
            circle_start = robot.getTime()
            circle_duration = 3.5  # seconds — adjust if rotation speed / sim timing different
            rotate_in_place(2.0, "left")
            while robot.step(TIMESTEP) != -1 and (robot.getTime() - circle_start) < circle_duration:
                # keep rotating; allow sensors to update during rotation
                # show camera while rotating (visual feedback already above)
                pass
            stop_robot()
            did_circle = True
            print(" Circle complete. Approaching charging station to start charging...")
            print("Robot needs charging — moving near black station...")
            # continue the loop so next iteration will do fine alignment/approach

        # Normal alignment + approach behavior (after circle or if already did circle)
        if black_pixels > BLACK_PIXEL_COUNT_THRESHOLD:
            error_x = cx - cam_width // 2
            # small centering tolerance
            if abs(error_x) < 20:
                move_forward(3.0)
                print("➡ Moving straight to charging station...")
            elif error_x < 0:
                rotate_in_place(2.0, "left")
                print("↪ Adjusting left...")
            else:
                rotate_in_place(2.0, "right")
                print("↩ Adjusting right...")
        else:
            # if no black area, keep slowly scanning
            rotate_in_place(1.5, "left")
            print(" Searching for black area...")

        # Dock detection (strong black presence)
        if black_pixels > 8000:
            stop_robot()
            print(" Docked on black charging station.")
            cv2.destroyAllWindows()
            return True

        # timeout
        if robot.getTime() - start_time > timeout:
            stop_robot()
            print(" Timeout: black not found.")
            cv2.destroyAllWindows()
            return False

    cv2.destroyAllWindows()
    return False

def go_to_charge_and_charge():
    global battery_level, charging
    print(" Battery low — navigating to charging station...")
    found = search_and_move_to_black()
    if not found:
        charging = False
        print(" Charging station not found.")
        return False

    charging = True
    print("⚡ Charging started.")
    while robot.step(TIMESTEP) != -1 and battery_level < BATTERY_MAX:
        battery_level = min(BATTERY_MAX, battery_level + CHARGE_STEP)
        print(f" Charging: {battery_level:.0f}%")
        if battery_level >= BATTERY_MAX:
            break
    charging = False
    stop_robot()
    print(" Battery fully charged.")
    return True

# ---------------- TASKS ----------------
def color_detection_task():
    print(" [TASK] Color detection running...")
    image = camera.getImage()
    bgr = webots_image_to_bgr(image)
    if bgr is not None:
        cv2.imshow("Camera View", bgr)
        cv2.waitKey(1)
    move_forward(2.0)

# ---------------- MAIN LOOP ----------------
def main_loop():
    global battery_level, current_task, last_task, charging
    print(" Starting TIAGo Autonomous Controller with full sensors.")
    last_update = robot.getTime()

    while robot.step(TIMESTEP) != -1:
        if battery_level <= BATTERY_THRESHOLD and not charging:
            print("\n Low battery detected!")
            last_task = current_task
            success = go_to_charge_and_charge()
            if success:
                current_task = last_task
            else:
                print("[INFO] Charging failed — retrying later.")
                continue

        # Task execution
        if current_task == "color_detection":
            color_detection_task()
        elif current_task == "obstacle_avoidance":
            obstacle_avoidance_task()
        else:
            move_forward(2.0)

        # Battery drain
        battery_level = max(0.0, battery_level - BATTERY_DRAIN_PER_STEP)

        # Status update
        if robot.getTime() - last_update > 1.0:
            print(f" Battery: {battery_level:.1f}% | Task: {current_task}")
            last_update = robot.getTime()

        # Random task switch
        if not charging and random.random() < 0.05:
            current_task = random.choice(["color_detection", "obstacle_avoidance"])
            print(f" Switching to new task: {current_task}")

    print("[INFO] Controller stopped.")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
