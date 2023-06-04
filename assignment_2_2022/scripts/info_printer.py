#! /usr/bin/env python3

"""
.. module:: info_printer
  :platform: Unix
  :synopsis: Python ROS 'info_printer' node module for RT1 Second Assignment

.. moduleauthor:: Teodoro Lima teoxlima1999@gmail.com

The "info_printer" node subscribes to the robot's position and velocity using a custom message and prints the distance of the robot from the target position and the average speed of the robot. 
It allows for a parameter to be set to control the frequency at which the node publishes the information. 
The node initializes the "info_printer" ROS node, subscribes to the "/pos_vel" topic to receive the position and velocity messages, and calls the callback function "PosVel" to process the received data. 
The callback function calculates the distance between the robot's current position and the desired destination position and calculates the average speed of the robot. 
The calculated values are then printed on the screen. The node continues to run until it is shut down.

Subscriber:
	/pos_vel
"""

import rospy
import math
import time
from scipy.spatial import distance #math.dist gets error in some computers
from assignment_2_2022.msg import position_velocity

InfoFreq = 1.0 #Frequency for printing infos
InfoPrinted = 0 #The last time that infos were printed


#Callback function
def PosVel(msg):
	"""
	The "PosVel" function is the callback function for the subscriber. 
	It takes in a message of type "position_velocity" and converts the time into milliseconds. 
	The function calculates the distance between the robot's current position and the desired destination position using the Euclidean distance formula. 
	It also calculates the average speed of the robot by calculating the magnitude of the velocity vector. 
	The calculated distance and average speed values are then printed on the screen if the specified time interval has elapsed since the last printing. 
	The function uses global variables to keep track of the printing frequency and the time of the last printing.

	Args:
	message(position_velocity)
	"""
	global InfoFreq, InfoPrinted
	Period=(1.0/InfoFreq)*1000 #Time in milliseconds
	CurrentTime=time.time()*1000

	if (CurrentTime-InfoPrinted)>Period:

		CurrentX=msg.CurrentX #Current position
		CurrentY=msg.CurrentY
		DestinationX=rospy.get_param("des_pos_x") #Position by user
		DestinationY=rospy.get_param("des_pos_y")
		
		CurrentPosition=(CurrentX,CurrentY)
		DestinationPosition=(DestinationX,DestinationY)
		Distance=distance.euclidean(DestinationPosition,CurrentPosition) #Euclidean distance

		Speed=math.sqrt(msg.VelX**2+msg.VelY**2) #Average speed

		print("Average speed: ",round(Speed,6)," Distance from selected position: ",round(Distance,6))

		InfoPrinted=CurrentTime
	

def main():
	"""
	The "main" function is the entry point for the node. 
	It initializes the ROS node with the name "info_printer". It retrieves the value of the frequency parameter, which determines how often the information is printed. 
	The function subscribes to the "/pos_vel" topic to receive the position and velocity messages and associates the callback function "PosVel" with the subscriber. 
	The function then enters a spin loop to keep the node running and processing incoming messages.
	"""
	rospy.init_node('info_printer')
	
	global InfoFreq
	InfoFreq=rospy.get_param("freq") #Get the publish frequency
	
	SubPosVel=rospy.Subscriber("/pos_vel", position_velocity,PosVel) #Get from "pos_vel" a parameter
	
	rospy.spin()


if __name__=="__main__":
	main()	
