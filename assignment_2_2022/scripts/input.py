#!/usr/bin/env python3

"""
.. module:: input
  :platform: Unix
  :synopsis: Python ROS 'input' node module for RT1 Second Assignment

.. moduleauthor:: Teodoro Lima teoxlima1999@gmail.com

The "input" node implements an action client that allows the user to set a target (x, y) or cancel it for a robot. 
This node plays a vital role in interacting with the robot, enabling users to specify a specific destination or abort an ongoing action.

Additionally, this node publishes the robot's position and velocity as a custom message. 
This custom message, containing fields (x, y, vel_x, vel_z), is continuously transmitted by the node, relying on the values published on the topic "/odom". 
The information regarding the robot's position and velocity is essential for other components in the system.

Thanks to the "input" node, real-time monitoring of the robot's position and velocity is made possible. 
The information provided by this node allows other components to take appropriate actions based on the current state of the robot.

Publisher:
  /pos_vel

Subscriber:
  /odom

ActionClient:
  /reaching_goal
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import position_velocity
import sys
import select

def PublishValues(msg):
  """
  The "PublishValues" function serves as a callback function that publishes the robot's position and velocity as a custom message type called "Position_velocity" on the "/pos_vel" topic. 
  It takes a parameter "msg" representing the message containing the robot's position and velocity. 
  The function creates a custom message object "PosVel" of type "position_velocity" and assigns the current position and velocity values obtained from the input message. 
  Finally, the function publishes the custom message on the "/pos_vel" topic.
  """
  global Pub

  Position = msg.pose.pose.position #Get the position
  Velocity = msg.twist.twist.linear	#Get the twist

  PosVel = position_velocity() #Create custom message

  PosVel.CurrentX=Position.x
  PosVel.CurrentY=Position.y
  PosVel.VelX=Velocity.x
  PosVel.VelY=Velocity.y

  Pub.publish(PosVel) #Publish the custom message


def ClientFunc():
  """
  The "ClientFunc" function creates an action client and waits for the server to become active. 
  It repeatedly prompts the user to input x and y positions as a goal for the robot. 
  If the user enters "c" within 10 seconds, the goal is cancelled; otherwise, the goal is sent to the server.
  """
  Client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction) #Create the action client
  Client.wait_for_server()

  while not rospy.is_shutdown():
    #Get the coordinates from keyboard
    DestinationX=0
    DestinationY=0
    while True:
      try:
        DestinationX = float(input("Position X: "))
        DestinationY = float(input("Position Y: "))     
      except ValueError:
        print("Please, insert numbers.\n")
        continue
      else:
        break 
  
    #Create the goal for the robot
    goal = assignment_2_2022.msg.PlanningGoal()
    goal.target_pose.pose.position.x = DestinationX
    goal.target_pose.pose.position.y = DestinationY

    Client.send_goal(goal) #Send the goal to the server
    
    #The user has 10 seconds in order to cancel the goal by typing 'c'
    print("Enter 'c' to cancel the goal:")
    val = select.select([sys.stdin], [], [], 10)[0]
    if val:
      value = sys.stdin.readline().rstrip()
      if (value == "c"):
        print("Goal cancelled!")
        Client.cancel_goal()

def main():
  """
  The "main" function acts as the starting point for the node. 
  It initializes the ROS node with the name "input" and creates a publisher to send the "position_velocity" message on the "/pos_vel" topic. 
  Additionally, it creates a subscriber to receive the robot's position and velocity from the "/odom" topic and invokes the "PublishValues" function as a callback. 
  Lastly, the function calls the "ClientFunc" function to handle user input for goal positions and goal cancellation.
  """
  rospy.init_node('input')

  global Pub
  Pub=rospy.Publisher("/pos_vel",position_velocity,queue_size=1) #Send a message with velocity and position

  SubOdom=rospy.Subscriber("/odom",Odometry,PublishValues) #Get from "Odom" velocity and position
  
  ClientFunc()

if __name__=='__main__':
    main()
