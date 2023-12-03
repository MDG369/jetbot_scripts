#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist

JETBOT_WHEEL_BASE=0.12 # Wheel base in m
JETBOT_MAX_SPEED=1 # m/s
last_received=0 # for measuring time between messages
# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

# geometry_twist msgs
def on_cmd_twist(msg):
	# Extract linear and angular velocities from the message
	linear = msg.linear.x
	angular = msg.angular.z
	rospy.loginfo(rospy.get_caller_id() + ' Linear speed=%s, Angular speed=%s', msg.linear, msg.angular)

	# Calculate wheel speeds in m/s
	left_speed = linear - angular*JETBOT_WHEEL_BASE/2
	right_speed = linear + angular*JETBOT_WHEEL_BASE/2
	rospy.loginfo(rospy.get_caller_id() + ' Left speed=%s, Right speed=%s', left_speed, right_speed)
	# Ideally we'd now use the desired wheel speeds along
	# with data from wheel speed sensors to come up with the
	# power we need to apply to the wheels, but we don't have
	# wheel speed sensors. Instead, we'll simply convert m/s
	# into percent of maximum wheel speed, which gives us a
	# duty cycle that we can apply to each motor.
	left_speed_percent = (100 * left_speed/JETBOT_MAX_SPEED)
	right_speed_percent = (100 * right_speed/JETBOT_MAX_SPEED)

	set_speed(motor_left_ID,  left_speed*3)
	set_speed(motor_right_ID,  right_speed*3) 
	# Publish the twist back to cmd_vel so the turtlebot moves in the sim 
	pub = rospy.Publisher('cmd_vel', Twist)
	pub.publish(msg)





# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)



# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
	rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
	rospy.Subscriber('~cmd_twist', Twist, on_cmd_twist)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

