#! /usr/bin/env python3

"""
.. module:: service
  :platform: Unix
  :synopsis: Python ROS 'service' node module for RT1 Second Assignment

.. moduleauthor:: Teodoro Lima teoxlima1999@gmail.com

The "service" is node that tracks the number of goals reached and cancelled within the system. 
It provides information about the total number of goals successfully achieved and the number of goals that have been cancelled, 
allowing users to monitor the effectiveness of goal-reaching operations in the system.
  
Service:
	service
    
Subscriber:
    /reaching_goal/result
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from assignment_2_2022.srv import goals, goalsResponse



GoalsReached = 0 #Counts reached goals
GoalsCancelled = 0 #Counts cancelled goals


#Sends to the Subscriber the goals info
def Results(msg):

	"""
	The "Results" function is a callback function that checks the status of the robot and increments the corresponding counters for the number of goals reached and cancelled.
	This function is invoked whenever a message related to goal results is received from the "/reaching_goal/result" topic. 
	If the goal status is 2, the counter for cancelled goals ("GoalsCancelled") is incremented. 
	If the goal status is 3, the counter for reached goals ("GoalsReached") is incremented.
	"""
	Status=msg.status.status

	global GoalsCancelled, GoalsReached
	if Status==2: #Cancelled goal (status=2)
		GoalsCancelled+=1
	elif Status==3: #Reached goal (status=3)
		GoalsReached+=1


#Service function
def data(req):

	"""
	The "data" function is a service function that returns the values of the counters for reached and cancelled goals. 
	When called, it returns a response object containing the total number of reached goals ("GoalsReached") and the number of cancelled goals ("GoalsCancelled").
	"""
	global GoalsCancelled, GoalsReached
	return goalsResponse(GoalsReached, GoalsCancelled)


def main():

	"""
	The "main" function is the main function of the node. It initializes the ROS node with the name "service" and creates a service to handle client requests. 
	Additionally, it creates a subscriber that receives goal results from the "/reaching_goal/result" topic and invokes the callback function "Results" to update the counters for reached and cancelled goals.
	"""
	rospy.init_node('service')
	
	srv=rospy.Service('service',goals,data)
	
	SubResults=rospy.Subscriber('/reaching_goal/result',assignment_2_2022.msg.PlanningActionResult,Results)

	rospy.spin()


if __name__=="__main__":
    main()
