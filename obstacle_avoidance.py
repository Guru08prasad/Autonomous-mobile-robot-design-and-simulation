from controller import Robot, DistanceSensor, Motor

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28
DISTANCE_THRESHOLD = 0.30  # 30 cm

# Initialize the Robot
robot = Robot()

# Get Distance Sensors
left_ds = robot.getDevice('left_ds')
right_ds = robot.getDevice('right_ds')

# Enable Distance Sensors
left_ds.enable(TIME_STEP)
right_ds.enable(TIME_STEP)

# Get Motors
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

# Set Motors to Velocity Control Mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main Control Loop
while robot.step(TIME_STEP) != -1:
    # Read Distance Sensors
    left_distance = left_ds.getValue()
    right_distance = right_ds.getValue()

    # Convert distance sensor readings to meters
    left_distance_m = left_distance / 1000.0
    right_distance_m = right_distance / 1000.0

    # Obstacle Avoidance Logic
    if left_distance_m < DISTANCE_THRESHOLD and right_distance_m < DISTANCE_THRESHOLD:
        # Both sensors detect an obstacle - move backwards
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
    elif left_distance_m < DISTANCE_THRESHOLD:
        # Left sensor detects an obstacle - turn right
        left_speed = MAX_SPEED
        right_speed = -MAX_SPEED
    elif right_distance_m < DISTANCE_THRESHOLD:
        # Right sensor detects an obstacle - turn left
        left_speed = -MAX_SPEED
        right_speed = MAX_SPEED
    else:
        # No obstacles - move forward
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED

    # Set motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
