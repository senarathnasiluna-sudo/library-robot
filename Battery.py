from controller import Robot

# Create robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get wheel motors (TIAGo base has wheels)
left_wheel = robot.getDevice("wheel_left_joint")
right_wheel = robot.getDevice("wheel_right_joint")

# Set motors to velocity control
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))

# Initial battery
battery_level = 100.0  

# States
STATE_NORMAL = "normal"
STATE_SEARCH = "search"
STATE_DEAD = "dead"

current_state = STATE_NORMAL

while robot.step(timestep) != -1:
    # Battery consumption
    if current_state != STATE_DEAD:
        battery_level -= 0.1  

    if battery_level < 0:
        battery_level = 0

    print(f"Battery: {battery_level:.1f}% | State: {current_state}")

    # ---- State Transitions ----
    if battery_level <= 35 and current_state == STATE_NORMAL:
        current_state = STATE_SEARCH
        print("⚠ Battery low! Switching to SEARCH state...")

    if battery_level == 0 and current_state != STATE_DEAD:
        current_state = STATE_DEAD
        print("❌ Battery empty! Robot shutting down...")

    # ---- Behaviors ----
    if current_state == STATE_NORMAL:
        # Move forward slowly
        left_wheel.setVelocity(2.0)
        right_wheel.setVelocity(2.0)

    elif current_state == STATE_SEARCH:
        # Rotate in place (search for charger)
        left_wheel.setVelocity(2.0)
        right_wheel.setVelocity(-2.0)

    elif current_state == STATE_DEAD:
        # Stop all movement
        left_wheel.setVelocity(0.0)
        right_wheel.setVelocity(0.0)
