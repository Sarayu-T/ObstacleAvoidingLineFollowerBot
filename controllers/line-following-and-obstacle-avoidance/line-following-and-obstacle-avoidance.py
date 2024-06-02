"""line_following_with_obstacle_avoidance controller."""
# This program implements a state-machine-based line-following behavior
# for the e-puck robot, with obstacle avoidance.

from controller import Robot, DistanceSensor, Motor

# Initialize variables
TIME_STEP = 64
MAX_SPEED = 6.28
COUNTER_MAX = 5

# create the Robot instance.
robot = Robot()
distance_sensor = robot.getDevice("distance sensor 2") 
time_step = 64  # This should match your simulation time step
distance_sensor.enable(time_step)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
distance_sensor.enable(time_step)

# states
states = ['forward', 'turn_right', 'turn_left']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0

# Initialize devices
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop
while robot.step(timestep) != -1:
    # Update sensor readings
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data
    line_right = gsValues[0] > 600
    line_left = gsValues[2] > 600

    # Implement the line-following state machine
    if current_state == 'forward':
        # Action for the current state: update speed variables
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED

        # check if it is necessary to update current_state
        if line_right and not line_left:
            current_state = 'turn_right'
            counter = 0
        elif line_left and not line_right:
            current_state = 'turn_left'
            counter = 0

    if current_state == 'turn_right':
        # Action for the current state: update speed variables
        leftSpeed = 0.8 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'

    if current_state == 'turn_left':
        # Action for the current state: update speed variables
        leftSpeed = 0.4 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'

    OBSTACLE_THRESHOLD = 1000
    distance_value = distance_sensor.getValue()
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Check if obstacle is detected
    if distance_value < OBSTACLE_THRESHOLD:
        # Stop and turn right to avoid obstacle
        leftSpeed = 0.8* MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED
    else:
        # Continue moving forward
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
 
    

    # increment counter
    counter += 1

    # Print current state and counter for debugging
    print('Counter:', counter, 'Current state:', current_state)
    distance_value = distance_sensor.getValue()
    print("Distance sensor value:", distance_value)

    # Set motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
