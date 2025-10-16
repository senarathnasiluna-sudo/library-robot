from controller import Robot, DistanceSensor, Lidar
import random

# ====== PARAMETERS ======
TIME_STEP = 64
MAX_SPEED = 3.0
OBSTACLE_THRESHOLD = 1.0  # Distance threshold for obstacle detection (meters)
CRITICAL_THRESHOLD = 0.3  # Critical distance for emergency stop

# ====== INIT ROBOT ======
robot = Robot()

# Motors (TIAGo base wheels)
left_motor = robot.getDevice("wheel_left_joint")
right_motor = robot.getDevice("wheel_right_joint")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Try to get LiDAR sensor first (more reliable)
lidar = None
try:
    lidar = robot.getDevice("Hokuyo URG-04LX-UG01")
    lidar.enable(TIME_STEP)
    print("✓ LiDAR sensor enabled")
except:
    print("✗ LiDAR sensor not found")

# Distance sensors for obstacle detection (TIAGo actual sensor names)
sensors = []
# Common TIAGo sensor names to try
sensor_names = [
    "base_sonar_01", "base_sonar_02", "base_sonar_03", "base_sonar_04",
    "sonar_01", "sonar_02", "sonar_03", "sonar_04",
    "base_laser_01", "base_laser_02", "base_laser_03",
    "front_laser", "left_laser", "right_laser", "back_laser"
]

# Initialize available sensors
for name in sensor_names:
    try:
        sensor = robot.getDevice(name)
        if sensor is not None:
            sensor.enable(TIME_STEP)
            sensors.append(sensor)
            print(f"✓ Enabled sensor: {name}")
    except:
        continue

print(f"Total distance sensors enabled: {len(sensors)}")

# If no distance sensors found, we'll rely on LiDAR
if len(sensors) == 0 and lidar is None:
    print("⚠ No sensors found! Robot will move randomly.")

# ====== FUNCTIONS ======
def move(left_speed, right_speed):
    """Set motor velocities"""
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

def get_lidar_data():
    """Get obstacle detection from LiDAR"""
    if lidar is None:
        return False, False, False
    
    try:
        ranges = lidar.getRangeImage()
        if not ranges:
            return False, False, False
        
        # Convert lidar data to obstacle detection
        num_points = len(ranges)
        if num_points == 0:
            return False, False, False
        
        # Divide LiDAR field of view into zones
        front_start = int(num_points * 0.4)  # Front 20% of field
        front_end = int(num_points * 0.6)
        left_start = int(num_points * 0.6)   # Left side
        left_end = int(num_points * 0.8)
        right_start = int(num_points * 0.2)  # Right side  
        right_end = int(num_points * 0.4)
        
        # Check for obstacles in each zone
        front_obstacle = any(r < OBSTACLE_THRESHOLD for r in ranges[front_start:front_end] if r != float('inf'))
        left_obstacle = any(r < OBSTACLE_THRESHOLD for r in ranges[left_start:left_end] if r != float('inf'))
        right_obstacle = any(r < OBSTACLE_THRESHOLD for r in ranges[right_start:right_end] if r != float('inf'))
        
        return front_obstacle, left_obstacle, right_obstacle
    
    except:
        return False, False, False

def get_sensor_values():
    """Get all distance sensor readings"""
    values = []
    for sensor in sensors:
        try:
            value = sensor.getValue()
            # Convert to meters if needed (some sensors return mm)
            if value > 1000:  # Likely in mm
                value = value / 1000.0
            values.append(value)
        except:
            values.append(float('inf'))
    return values

def detect_obstacles():
    """Check if obstacles are detected using available sensors"""
    
    # First try LiDAR if available
    if lidar is not None:
        return get_lidar_data()
    
    # Fall back to distance sensors
    sensor_values = get_sensor_values()
    
    if not sensor_values:
        return False, False, False  # front, left, right
    
    # Simple obstacle detection based on available sensors
    front_obstacle = False
    left_obstacle = False  
    right_obstacle = False
    
    # If we have sensors, check them
    if len(sensor_values) >= 1:
        # At least one sensor - use for front detection
        front_obstacle = min(sensor_values) < OBSTACLE_THRESHOLD
    
    if len(sensor_values) >= 2:
        # Two sensors - front and back, or left and right
        front_obstacle = sensor_values[0] < OBSTACLE_THRESHOLD
        # Use second sensor for side detection
        if sensor_values[1] < OBSTACLE_THRESHOLD:
            # Randomly assign to left or right if we don't know the arrangement
            if random.choice([True, False]):
                left_obstacle = True
            else:
                right_obstacle = True
    
    if len(sensor_values) >= 3:
        # Three sensors - likely front, left, right arrangement
        front_obstacle = sensor_values[0] < OBSTACLE_THRESHOLD
        left_obstacle = sensor_values[1] < OBSTACLE_THRESHOLD
        right_obstacle = sensor_values[2] < OBSTACLE_THRESHOLD
    
    if len(sensor_values) >= 4:
        # Four sensors - better arrangement
        front_obstacle = min(sensor_values[0:2]) < OBSTACLE_THRESHOLD
        left_obstacle = sensor_values[2] < OBSTACLE_THRESHOLD
        right_obstacle = sensor_values[3] < OBSTACLE_THRESHOLD
    
    return front_obstacle, left_obstacle, right_obstacle

def obstacle_avoidance():
    """Enhanced obstacle avoidance behavior"""
    front_obstacle, left_obstacle, right_obstacle = detect_obstacles()
    
    # Get minimum distance for speed control
    all_values = get_sensor_values()
    min_distance = min(all_values) if all_values else OBSTACLE_THRESHOLD + 1
    
    # Emergency stop for critical distance
    if min_distance < CRITICAL_THRESHOLD:
        move(0, 0)
        print("🛑 EMERGENCY STOP - Obstacle too close!")
        return
    
    # Adjust speed based on closest obstacle
    speed_factor = min(1.0, min_distance / OBSTACLE_THRESHOLD)
    current_speed = MAX_SPEED * max(0.3, speed_factor)  # Minimum 30% speed
    
    if front_obstacle:
        print(f"🚧 Front obstacle detected! Distance: {min_distance:.2f}m")
        # Choose direction based on side sensors
        if not right_obstacle:
            # Turn right
            move(-current_speed * 0.5, current_speed * 0.8)
            print("↪ Turning right to avoid obstacle")
        elif not left_obstacle:
            # Turn left  
            move(current_speed * 0.8, -current_speed * 0.5)
            print("↩ Turning left to avoid obstacle")
        else:
            # Both sides blocked, reverse and turn
            move(-current_speed * 0.4, current_speed * 0.6)
            print("⬅ Backing up and turning right")
    
    elif left_obstacle:
        print(f"🚧 Left obstacle detected! Distance: {min_distance:.2f}m")
        # Turn right
        move(current_speed * 0.9, current_speed * 0.5)
        print("↪ Steering away from left obstacle")
    
    elif right_obstacle:
        print(f"🚧 Right obstacle detected! Distance: {min_distance:.2f}m")
        # Turn left
        move(current_speed * 0.5, current_speed * 0.9)
        print("↩ Steering away from right obstacle")
    
    else:
        # No obstacles, move forward
        move(current_speed, current_speed)
        print(f"✅ Path clear - Moving forward at {current_speed:.1f}")

def print_sensor_status():
    """Print detailed sensor status"""
    sensor_values = get_sensor_values()
    front_obs, left_obs, right_obs = detect_obstacles()
    
    print(f"Sensors: {len(sensors)} distance sensors, LiDAR: {'Yes' if lidar else 'No'}")
    if sensor_values:
        print(f"Distance readings: {[f'{val:.2f}m' for val in sensor_values[:4]]}")
    print(f"Obstacles - Front: {front_obs}, Left: {left_obs}, Right: {right_obs}")

# ====== MAIN LOOP ======
step_count = 0
print("🤖 Starting TIAGo Enhanced Obstacle Avoidance...")
print(f"Obstacle threshold: {OBSTACLE_THRESHOLD}m")
print(f"Critical threshold: {CRITICAL_THRESHOLD}m")
print("=" * 60)

while robot.step(TIME_STEP) != -1:
    step_count += 1
    
    # Print status every 100 steps
    if step_count % 100 == 0:
        print(f"\n--- Step {step_count} | Time: {step_count * TIME_STEP / 1000:.1f}s ---")
        print_sensor_status()
        print("-" * 50)
    
    # Execute obstacle avoidance
    obstacle_avoidance()