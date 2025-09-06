#!/usr/bin/env python3
import sys
import pygame

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.node  import Node
from rclpy.qos   import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class xbox (Node):

	def __init__(self):
		super().__init__('poc_xbox')
        
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
			)
		
		#===============================================================================================
		# publisher nodes
		#
		self.stick_pub    = self.create_publisher (geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
		self.actions_pub  = self.create_publisher (std_msgs.msg.UInt8, 	    '/action_message', 	      qos_profile)

		#================================== 
		# create timers
		#
		timer_period = 0.04  # seconds
		self.timer = self.create_timer(timer_period, self.xbox_loop_callback)
		
		#================================== 
		# init & configure
		#
		pygame.init()
		pygame.joystick.init()

		#================================== 
		# variable & classes
		#
		self.joystick    = pygame.joystick.Joystick(0)
		self.actions_msg = std_msgs.msg.UInt8()
		self.twist       = geometry_msgs.msg.Twist()
		self.yaw_val     = 0
		
		self.m3_vs_m2    = 1  	# if {m3_vs_m2 = 1} => mode 3 is active
                                # if {m3_vs_m2 = 0} => mode 2 is active
		
	def xbox_loop_callback(self):
		
		self.actions_msg.data = 0 
	
		for event in pygame.event.get():
			if event.type == pygame.JOYBUTTONDOWN:
				if event.button == 0:
					self.actions_msg.data = self.actions_msg.data | 2
					self.get_logger().info("[A] Land sent!")
				if event.button == 1:
					self.get_logger().info("[B]")
				if event.button == 2:
					self.get_logger().info("[X]")
				if event.button == 3:
					self.actions_msg.data = self.actions_msg.data | 1
					self.get_logger().info("[Y] Takeoff sent!")
				if event.button == 4:
					self.get_logger().info("[LB]")
				if event.button == 5:
					self.get_logger().info("[RB]")
				if event.button == 6:
					self.get_logger().info("[Left Special]")
				if event.button == 7:
					self.get_logger().info("[Right Special]")
				if event.button == 8:
					self.get_logger().info("[XBOX Button]")
				if event.button == 9:
					self.get_logger().info("[LS]")
				if event.button == 10:
					self.get_logger().info("[RS]")

				self.actions_pub.publish(self.actions_msg)


		stick1 = (self.joystick.get_axis(0), self.joystick.get_axis(1))
		stick2 = (self.joystick.get_axis(3), self.joystick.get_axis(4))

		# LT     = self.joystick.get_axis(2)
		# RT     = self.joystick.get_axis(5)
		# dpad   = self.joystick.get_hat(0)
		
		if self.m3_vs_m2 == 1:
			x_val   = - stick1[1]
			y_val   = - stick1[0]
			z_val   = - stick2[1]
			self.yaw_val =  stick2[0]
		else:
			x_val   = - stick2[1]
			y_val   = - stick2[0]
			z_val   = - stick1[1]
			self.yaw_val =  stick1[0]	
			
		# self.get_logger().info(f"x: {x_val} - y: {y_val}")
		# self.get_logger().info(f"z: {z_val} - yaw: {yaw_val}")
		
		self.twist.linear.x = x_val
		self.twist.linear.y = y_val
		self.twist.linear.z = z_val
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = self.yaw_val	
		
		self.stick_pub.publish(self.twist)	

def main(args=None):
	rclpy.init(args=args)

	xbox_control = xbox()

	rclpy.spin(xbox_control)

	xbox_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
