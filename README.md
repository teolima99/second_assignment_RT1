Research Track I - Assignment 2
================================

The goal of this project is to implement the bug0 algorithm to enable a mobile robot to navigate a 3D space. The robot's objective is to reach specified target coordinates while effectively avoiding obstacles within a virtual environment simulated using Gazebo, the standard virtual simulator of ROS.

The project requires the creation of three nodes within the new package, each serving a distinct purpose. Here is an overview of their functionalities:

* Action Client Node:
  This node acts as an action client, allowing users to set target positions (x, y) for the robot or cancel the current target. Additionally, it publishes the robot's position and velocity as a       custom message (x, y, vel_x, vel_z) by utilizing the values published on the /odom topic. If necessary, you have the flexibility to develop two separate nodes: one for the user interface and       another for publishing the custom message.
  
* Service Node:
  This node implements a service that, upon invocation, prints the number of goals reached and the number of goals cancelled. It serves as a convenient way to obtain this information.
  
* Position and Velocity Subscriber Node:
  This node subscribes to the robot's position and velocity using the custom message. It calculates and prints the distance between the robot and the target position, as well as the robot's average   speed. You can adjust the publishing frequency of this information using a parameter.
  
In addition to the nodes, the package also includes a launch file to simplify the setup of the entire simulation. This launch file initializes all the required nodes and allows you to configure the publishing frequency for the information published by node (c).



![default_gzclient_camera(1)-2023-01-27T15_23_29 351549](https://user-images.githubusercontent.com/117213899/215110545-c90c0ec1-ce38-4ce9-86e4-545e46050bc1.jpg)

Installing and running
----------------------
To run the program, we have to install xterm:
```bash
$ sudo apt-get install xterm
```
and SciPy:
```bash
$ sudo apt-get install python3-scipy
```
Go inside the src folder of your ROS workspace and clone the repository folder:
```bash
$ git clone https://github.com/teolima99/second_assignment_RT1.git
```
Then, from the root directory of your ROS workspace, run the command:
```bash
$ catkin_make
```
You can run `$ roscore`  in a terminal or skip it. Anyway, it will be runned automatically. Run the string below to start the programme:
```bash
$ roslaunch assignment_2_2022 assignment2.launch
```

Troubleshooting
----------------------
Check your python version running:
```bash
$ python --version
```
If it appears Python 3, there is no problem. If it appears Python 2, run this:
```bash
$ sudo apt-get install python-is-python3
```

Nodes
----------------------
Inside `~/<your ros workspace folder>/src/assignment_2_2022/scripts/` there are 6 python files:

1. `bug_as.py`: The action server node receives the requested position from the client and invokes the necessary services to navigate the robot to the specified location.;
2. `input.py`: The action client node is responsible for prompting the user to input the X and Y coordinates of the desired final destination for the robot or delete them. Subsequently, it publishes the robot's position and speed as a custom message on the "/position_velocity" topic, using the values obtained from the "/odom" topic.
3. `printer.py`: The node prints the distance between the robot and the target position, as well as its average speed, on the terminal. These parameters are extracted from the "/position_velocity" topic as a custom message.
4. `go_to_point_service.py`: implementation of a service node. When called, it moves the robot to the requested position.
5. `wall_follow_service.py`: implementation of a service node. When called, it allows the robot to move around an obstacle (in our case a wall).
6. `service.py`: it is a service node. When called, it prints the number of successful reached targets and the number of cancelled targets.

Flowchart (`input.py`)
----------------------
![flowchart](https://github.com/teolima99/second_assignment_RT1/blob/471fce5c1253d4e519c3ff461f54285d43b85331/assignment_2_2022/flowchart.png)


Possible improvements
----------------------
Here are some ideas for future improvements:
- By indicating the size of the arena, it would be possible to set a range of possible target coordinates;
- A graphic indicator could be added at the point corresponding to the entered coordinates to be reached.
