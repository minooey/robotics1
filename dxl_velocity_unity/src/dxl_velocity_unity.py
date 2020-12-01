#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import rospy
from std_msgs.msg import Int32, Bool
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class Controller:
	
	def __init__(self):
		# Control table address
		self.ADDR_OPERATING_MODE    = 11		  # Control table address for operating mode
		self.ADDR_PRO_TORQUE_ENABLE = 64               # Control table address is different in Dynamixel model
		self.ADDR_PRO_GOAL_VEL      = 104              # Control table address for goal velocity
		self.ADDR_PRO_PRESENT_VEL   = 128              # Control table address for present velocity
		self.ADDR_PRO_PRESENT_LOAD  = 126              # Control table address for present load
		self.ADDR_MOVING = 122

		# Protocol version
		PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# Default setting
		self.DXL_ID                      = 1                 # Dynamixel ID : 1
		BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
		DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
														# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(PROTOCOL_VERSION)
		
		#ROS Variables
		rospy.init_node('dxl_listener', anonymous=True)

		self.velocity_sub = rospy.Subscriber("set_velocity", Int32, self.callback)
		self.set_velocity = 0
		self.velocity_pub = rospy.Publisher("current_velocity", Int32, queue_size=10)
		self.torque_sub = rospy.Subscriber("set_torque", Bool, self.torqueCallback)
		self.torque_enable = 1
		self.state_pub = rospy.Publisher("motor_state", Bool, queue_size=10)
		
		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		# Set port baudrate
		if self.portHandler.setBaudRate(BAUDRATE):
			print("Succeeded to change the baudrate")
		else:
			print("Failed to change the baudrate")
			print("Press any key to terminate...")
			getch()
			quit()

	def dxl_operating_mode(self, operating_mode_value):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, operating_mode_value)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Velocity operating mode enabled.")

	def dxl_torque(self, value):
		# Enable Dynamixel Torque
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_TORQUE_ENABLE, value)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def dxl_write(self, sub_motor_vel_value):
		# Write goal velocity position
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_GOAL_VEL, sub_motor_vel_value)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))


	def callback(self, data):
		self.set_velocity = data.data
		
	def torqueCallback(self, data):
		self.torque_enable = data.data
			
	def dxl_loop(self):
		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.

		print("Press Ctrl + C  to quit!")

		#ROS Loop
		while not rospy.core.is_shutdown():
			#Write enable/disable torque
			self.dxl_torque(self.torque_enable)
			
			#Write velocity
			self.dxl_write(self.set_velocity)
		
			#Read velocity status
			dxl_present_vel, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_PRESENT_VEL)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			velocity_message = Int32()
			velocity_message.data = dxl_present_vel
			self.velocity_pub.publish(velocity_message)
			
			#Read moving status
			dxl_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MOVING)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			self.state_pub.publish(dxl_moving)
			rospy.rostime.wallsleep(0.1)

if __name__ == '__main__':
    c = Controller()
    
    c.dxl_operating_mode(1) # this is the operating mode value for velocity

    c.dxl_torque(1)

    c.dxl_loop()

    c.dxl_torque(0)
