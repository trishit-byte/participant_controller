from controller import Robot

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Obtain waypoints
waypoints = []
waypoints_string = robot.getCustomData()
waypoints_split = waypoints_string.split()
for i in range(10):
    waypoints_element = [float(waypoints_split[2*i]), float(waypoints_split[2*i+1])]
    waypoints.append(waypoints_element)
print('Waypoints:', waypoints)

# begin{please do not change}
if (len(waypoints_split) != 20):
    waypoints_string = ' '.join(waypoints_split[:20])
# end{please do not change}

# Initialize devices
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
arm_1 = robot.getDevice('arm_1_joint')
arm_2 = robot.getDevice('arm_2_joint')
arm_4 = robot.getDevice('arm_4_joint')
gps = robot.getDevice('gps')
gps_ee = robot.getDevice('gps_ee')
imu = robot.getDevice('inertial unit')

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

arm_1.setPosition(90.0*3.14159/180.0)
arm_2.setPosition(45.0*3.14159/180.0)
arm_4.setPosition(45.0*3.14159/180.0) # -15.0 or 45.0

gps.enable(timestep)
gps_ee.enable(timestep)
imu.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    imu_rads = imu.getRollPitchYaw()
    gps_vals = gps.getValues()
    # begin{please do not change}
    gps_ee_vals = gps_ee.getValues()
    robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
    # end{please do not change}

    print('Hello World from Python!', gps_vals, gps_ee_vals, [x*180.0/3.14159 for x in imu_rads])
    motor_left.setVelocity(5.0)
    motor_right.setVelocity(5.0)
    if gps_ee_vals[0] > 4.5:
        break

print('Bye from Python!')
