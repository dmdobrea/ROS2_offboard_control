#!/usr/bin/env python

# ros2 run takeoff_land keyb_cntrl
# ros2 run takeoff_land updown


import rclpy
import numpy as np

from rclpy.node  import Node
from rclpy.clock import Clock
from rclpy.qos   import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleCommand

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg      import UInt8

myPi  = 3.14159265358979
my2Pi = 6.283185307


class OffboardControl(Node):

	def __init__(self):
		super().__init__('poc_fly_xbox')
        
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
			)

		#================================== Create subscriptions
		# from PX4
		#
		self.status_sub   = self.create_subscription( VehicleStatus,        '/fmu/out/vehicle_status_v1',      self.vehicle_status_callback, qos_profile)
		
		# get real-time information of the UAV
		self.gps_sub     = self.create_subscription ( VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.gps_callback, qos_profile)
         
		# from teleop XBOX node
		self.offboard_velocity_sub = self.create_subscription( Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback,    qos_profile)
		self.my_action_sub         = self.create_subscription( UInt8, '/action_message',        self.action_message_callback,       qos_profile)

		#================================== Create publishers
		# commands like arm, takeoff
		#
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',                     10)
        
		# offboard mode: velocity, position, acceleration, etc.
		self.offboard_mode_publisher   = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',               qos_profile)
        
		# set the movement through: velocity, position, acceleration, etc.
		self.trajectory_publisher      = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',                 qos_profile)
        
		#================================== Create timers
		#
		#creates callback function for the missions timer
		# period is arbitrary, just should be more than 2Hz
		arm_timer_period = .1 # seconds
		self.arm_timer_ = self.create_timer(arm_timer_period, self.mission_timer_callback)

		# creates callback function for the command loop
		# period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency 
		# is recommended, commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
		self.cmd_timer_period = 0.01  # seconds
		self.timer = self.create_timer(self.cmd_timer_period, self.cmdloop_callback)

		#================================== Variables
		#
		self.velocity = Vector3()
		self.yaw      = 0.0  	# yaw value I send as command
		self.trueYaw  = 0.0  	# current yaw value of drone
		self.maxVxyz  = 6.0	    # in a normal case velocity belongs to [-1, 1], based on maxVxyz I will have [-6, 6]
		self.maxVyaw  = 2.0

		self.nav_state   = VehicleStatus.NAVIGATION_STATE_MAX
		self.arm_state   = VehicleStatus.ARMING_STATE_DISARMED    # ARMING_STATE_DISARMED = 1, ARMING_STATE_ARMED = 2
		self.failsafe    = False
		self.flightCheck = False        

		self.mission_ctrl      = 0		# Takeoff = 1, Land = 2, Hold = 4,
		self.mission_ctrl_old  = 0		# Takeoff = 1, Land = 2, Hold = 4,
		self.current_state     = "IDLE"
		
		self.myCnt = 0
		
		self.real_lat = 0.0
		self.real_lon = 0.0
		self.real_alt = 0.0

		# in the end go to Hold mode		
		self.mode_Hold()

	def gps_callback(self, msg):
		if msg.lat_lon_valid == True:
			self.real_lat = msg.lat
			self.real_lon = msg.lon
			self.real_alt = msg.alt
			# self.get_logger().info(f"GPS current position: lat={self.current_lat}, lon={self.current_lat}")
	
	def action_message_callback (self, msg):
    		self.mission_ctrl_old = self.mission_ctrl
    		self.mission_ctrl     = msg.data 		# Takeoff = 1, Land = 2, Hold = 4,
        
    		if self.mission_ctrl == 1:
    			self.get_logger().info("Takeoff received!")
        	
    		if self.mission_ctrl == 2:
    			self.get_logger().info("Land received!")	

	#callback function that arms, takes off, and switches to offboard mode
	#implements a finite state machine
	def mission_timer_callback(self):
	
		# Takeoff
		if (self.mission_ctrl == 1 and self.mission_ctrl_old != 2):	# enter in Takeoff if the UAV is not in landing mode
			# self.get_logger().info(" ")
	    		
			match self.current_state:
				case "IDLE":
					if (self.flightCheck and self.mission_ctrl == 1):
						self.current_state = "ARMING"
						self.get_logger().info(f"[1] Going to Arming")
		
						self.myCnt = 0
						
				case "ARMING":
					if(not(self.flightCheck)):
						self.current_state = "IDLE"
						self.get_logger().info(f"[2] Arming, Flight Check Failed going to IDLE")
				
					if(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 5):
						self.current_state = "TAKEOFF"
						self.get_logger().info(f"[2] Arming, Takeoff")
						
					elif (self.arm_state == VehicleStatus.ARMING_STATE_DISARMED):
						self.arm() 			#send arm command

				case "TAKEOFF":
					if(not(self.flightCheck)):
						self.current_state = "IDLE"
						self.get_logger().info(f"[3] Takeoff, Flight Check Failed")
						
					if(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
						self.current_state = "LOITER"
						self.get_logger().info(f"[3] Takeoff, Loiter")
						
					elif (self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
						self.take_off_full(5.0)		#send takeoff command - altitude positive value only
										
				case "LOITER":
					if(not(self.flightCheck)):
						self.current_state = "IDLE"
						self.get_logger().info(f"Loiter, Flight Check Failed")

					elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
						self.current_state = "OFFBOARD"
						self.get_logger().info(f"[4] Auto Loiter")	
					self.arm() 						
				
				case "OFFBOARD":
					if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
						self.current_state = "IDLE"
						self.get_logger().info(f"Offboard, Flight Check Failed")
						
					if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):				
						self.mode_Offboard()
						self.get_logger().info(f"[5] offboard mode active!")
						
						self.mission_ctrl  = 8		# Offboard
						self.current_state = "IDLE"

			self.myCnt += 1

		# Land
		if (self.mission_ctrl == 2):
			# self.get_logger().info(" ")

			if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
				self.mode_Hold()
		
			if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
				self.mode_land()
				
			if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
				self.mode_Hold()
				
				self.current_state = "IDLE"
				self.mission_ctrl  = 4		# Hold

	# Arms the vehicle
	def arm(self):
        	self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        	self.get_logger().info("Arm command send")

	# Takes off the vehicle to a user specified altitude (meters)
	# in this mode eliminate "Already higher than takeoff altitude (not descending)" error
	def take_off_full(self, altitude):
        	self.publish_vehicle_command(
        		VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 
        		param5 = self.real_lat, 
        		param6 = self.real_lon,
        		param7 = (self.real_alt + altitude)) # param7 is altitude in meters
        	self.get_logger().info("Takeoff full command was send")
        	
	def take_off(self, altitude):
        	self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=altitude) # param7 is altitude in meters - param1 = 1.0 ?        	
        	self.get_logger().info("Takeoff command was send")

	def mode_auto_takeoff_mode(self):
		# param1 = 1.0  - PX4_CUSTOM_MAIN_MODE      = 1 (AUTO)
		# param2 = 2.0  - PX4_SUB_MODE_AUTO_TAKEOFF = 2
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=2.)
		self.get_logger().info("Go to Auto Takeoff mode")

	def mode_land(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
		self.get_logger().info("Land command was send")

	def mode_Offboard(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
		self.get_logger().info("Offboard command was send")

	def mode_Possition(self):
		# de verificat: self.publish_vehicle_command(
		#	VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
		#	1.0, 		# Custom mode enabled
		#	1.0		# PX4_MAIN_MODE_POSCTL
		#	)
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)   #  1.0 => Manual,   2.0 => Altitude,  3.0 => Possition
		self.get_logger().info("Possition mode command send")     

	# is working => tested in PX4 v1.16.0   MISSION MODE
	def mode_Mission(self):
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
			1.0, 		# enables custom mode (VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED)
			4.0, 		# PX4_CUSTOM_MAIN_MODE_AUTO
			0.0)
		self.get_logger().info("Mission mode has been set")

	# is working => tested in PX4 v1.16.0 
	def mode_Hold(self):
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
			1.0, 		# enables custom mode (VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED)
			4.0, 		# PX4_CUSTOM_MAIN_MODE_AUTO
			3.0             # PX4_CUSTOM_SUB_MODE_AUTO_LOITER
			) 
		self.get_logger().info("Auto loiter has been send")

	#publishes command to /fmu/in/vehicle_command
	def publish_vehicle_command(self, command, param1=0., param2=0., param3=0., param5=0., param6=0., param7=5.0):
        	msg = VehicleCommand()
        	
        	msg.param1 = param1
        	msg.param2 = param2
        	msg.param3 = param3
        	
        	msg.param5 = param5 		# param5 = latitude
        	msg.param6 = param6 		# param6 = longitude
        	msg.param7 = param7    		# altitude value in takeoff command
        	
        	msg.command = command  		# command ID - e.q. 22 VEHICLE_CMD_NAV_TAKEOFF
        	msg.target_system     = 1	# system which should execute the command
        	msg.target_component  = 1 	# component which should execute the command, 0 for all components
        	msg.source_system     = 1       # system sending the command
        	msg.source_component  = 1  	# component sending the command
        	msg.from_external     = True
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	
        	try:
        		self.vehicle_command_publisher.publish(msg)
        	except Exception as e:
        		self.get_logger().info(f"Failed to publish command: {e}")	

	#receives and sets vehicle status values 
	def vehicle_status_callback(self, msg):

        	if (msg.nav_state != self.nav_state):
        		self.get_logger().info(f"NAV_STATUS: {self.decodeNavState(msg.nav_state)}")
        
        	if (msg.arming_state != self.arm_state):
        		self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        	if (msg.failsafe != self.failsafe):
	        	self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        	if (msg.pre_flight_checks_pass != self.flightCheck):
        		self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        	self.nav_state   = msg.nav_state
        	self.arm_state   = msg.arming_state
        	self.failsafe    = msg.failsafe
        	self.flightCheck = msg.pre_flight_checks_pass

	def decodeNavState (self, navState):
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
        		return "AUTO TAKEOFF"
        	if (navState == VehicleStatus.NAVIGATION_STATE_POSCTL):
        		return "Position control mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
        		return "AUTO LOITER"
        	if (navState == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
        		return "OFFBOARD"
        	if (navState == VehicleStatus.NAVIGATION_STATE_MANUAL):
        		return "manual mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_ALTCTL):
        		return "altitude control mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION):
        		return "auto mission mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_RTL):
        		return "auto return to launch mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_ACRO):
        		return "Acrobat mode actives"        		
        	if (navState == VehicleStatus.NAVIGATION_STATE_DESCEND):
        		return "descend mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_TERMINATION):
        		return "termination mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_STAB):
        		return "stabilized mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
        		return "land mode"
        	if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET):
        		return "auto follow"
        	if (navState == VehicleStatus.NAVIGATION_STATE_ORBIT):
        		return "orbit in a circle"
        	else:
        		return navState

	#receives Twist commands from Teleop and converts NED -> FLU
	def offboard_velocity_callback(self, msg):
		# stick dead zone
		if  -0.09 < msg.linear.x  and msg.linear.x < 0.09:
			msg.linear.x = 0.0
			
		if  -0.09 < msg.linear.y  and msg.linear.y < 0.09:
			msg.linear.y = 0.0
	
		if  -0.09 < msg.linear.z  and msg.linear.z < 0.09:
			msg.linear.z = 0.0
			
		if  -0.09 < msg.angular.z and msg.angular.z < 0.09:
			msg.angular.z = 0.0		
		
		# self.get_logger().info(f"{msg.linear.x} : {msg.linear.y} : {msg.linear.z} : {msg.angular.z}")
		
        #implements FLU -> NED transformation
        # PX4 expects commands in the NED frame!
 
        # the following are correct only if we align both frames along the y axis 
        # X (FLU) is -X (NED)
		self.velocity.x =   msg.linear.x * self.maxVxyz

        # Y (FLU) is Y (NED)
		self.velocity.y = - msg.linear.y * self.maxVxyz

        # Z (FLU) is -Z (NED)
		self.velocity.z = -msg.linear.z * self.maxVxyz

		# A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
		self.yaw = self.yaw + msg.angular.z * self.cmd_timer_period * self.maxVyaw
		
		if self.yaw > myPi:
			self.yaw = self.yaw - my2Pi
		elif self.yaw < -myPi:
			self.yaw = self.yaw + my2Pi
        		 
	def NEDangle (self, angle):
		if angle > myPi:
			return (angle - my2Pi)
		if angle < -myPi:
			return (angle + my2Pi)
	
	#receives current trajectory values from drone and grabs the yaw value of the orientation
	def attitude_callback(self, msg):
		#   +179 N -179
		# +90(V)         -90(E) 
		#   +    S(0)    -
		orientation_q = msg.q

		#trueYaw is the drones current yaw value
		self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
			1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
			
		if self.trueYaw >= 0:
			self.trueYaw = self.trueYaw - myPi
		else:
			self.trueYaw = self.trueYaw + myPi
			
		# self.get_logger().info(f"Current yaw value of drone: {self.trueYaw * 180/3.1415926}")
        
	#publishes offboard control modes and velocity as trajectory setpoints
	def cmdloop_callback(self):
		if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
			self.newTrajectorySet(self.velocity.x, self.velocity.y, self.velocity.z, self.yaw, False)
		else:
			# important the following condition - if not used, after UAV land
			# will not go automaticly to VehicleStatus.ARMING_STATE_DISARMED
			# mainly because offboard message are send.
			if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
				self.newTrajectorySet(0.0, 0.0, 0.0, 0.0, False)

	def newTrajectorySet (self, x_SN, y_VE, z_Down, heading_angle, position = True):
		# Publish offboard control modes
		offboard_msg = OffboardControlMode()
		offboard_msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
		
		if position:
			offboard_msg.position     = True
			offboard_msg.velocity     = False
		else:
			offboard_msg.position     = False
			offboard_msg.velocity     = True
			
		offboard_msg.acceleration = False
		offboard_msg.attitude     = False
		offboard_body_rate        = False
		
		self.offboard_mode_publisher.publish(offboard_msg)	
		#===============================================================
	
		# NED local world frame
		# Publish the trajectory setpoints 
		trajectory_msg = TrajectorySetpoint()
		trajectory_msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
		
		if position:
			# X Position in meters (positive is forward or North)
			# Y Position in meters (positive is right or East)
			# Z Position in meters (positive is down)	
			trajectory_msg.position = [x_SN, y_VE, z_Down]
			
			# X velocity in m/s (positive is forward or North)
			# Y velocity in m/s (positive is right or East)
			# Z velocity in m/s (positive is down)
			trajectory_msg.velocity = [float("nan"), float("nan"), float("nan")]
		else:
			# Compute velocity in the world frame:
			# - the conversion of frames from FLU -> NED was done previously
			# - but the converted frame is rotaded with the trueYaw angle related to the NED
			cos_yaw = np.cos(self.yaw)
			sin_yaw = np.sin(self.yaw)
            
			velocity_world_x = (x_SN * cos_yaw - y_VE * sin_yaw)
			velocity_world_y = (x_SN * sin_yaw + y_VE * cos_yaw)
		
			# X Position in meters (positive is forward or North)
			# Y Position in meters (positive is right or East)
			# Z Position in meters (positive is down)	
			trajectory_msg.position = [float("nan"), float("nan"), float("nan")]
			
			# X velocity in m/s (positive is forward or North)
			# Y velocity in m/s (positive is right or East)
			# Z velocity in m/s (positive is down)
			trajectory_msg.velocity = [velocity_world_x, velocity_world_y, z_Down]	
		
		# Bearing in degrees from North (0° to 360°) or
		#         N (0) 
		# -90(V)         +90(E) 
		#   -179 S(180) +179
		trajectory_msg.yaw = heading_angle		# yaw or heading in radians (0 is forward or North)
		
		trajectory_msg.jerk[0]         = float("nan")
		trajectory_msg.jerk[1]         = float("nan")
		trajectory_msg.jerk[2]         = float("nan")		
		trajectory_msg.acceleration[0] = float("nan")	# X acceleration in m/s/s (positive is forward or North)
		trajectory_msg.acceleration[1] = float("nan")	# Y acceleration in m/s/s (positive is right or East)
		trajectory_msg.acceleration[2] = float("nan")	# Z acceleration in m/s/s (positive is down)
		trajectory_msg.yawspeed = 0.0			# yaw rate in rad/s
			
		self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
	rclpy.init(args=args)

	offboard_control = OffboardControl()

	rclpy.spin(offboard_control)

	offboard_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
