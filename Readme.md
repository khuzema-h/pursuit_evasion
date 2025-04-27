# ENPM 661 Project 5 : Pursuit and Evasion 
#### Project Members : Adil Qureshi (1220607905), Khuzema Habib(12188776), Sounderya Venugopal(121272423)

##  Project Description

In this project, we use an A star Path Planner to generate a trajectory for a given map in 3 different environments:

- 2D Environment
- Gazebo


We first run the 2D Path Planner to generate a trajectory for the other 2 simulations in 3D, applying clearances where required. The 2D Map was generated with the use of half planes and semi-algebraic equations in Matplotlib then visualized in opencv for trajectory generation. 

Link to Folder Containing Path Generation, Gazebo and Falcon Simulations can be found here : https://drive.google.com/drive/folders/1ESFB4iaVE5oencMvhOxJujY2T5UsuXSq?usp=drive_link

## Gazebo Trajectory Controller

The 2D path planner outputs the coordinates of the trajectory in a file called planner_waypoints.txt, which is then read by the trajectory controller.

The controller implements proportional control to follow the waypoints, subscribing to /odom to extract pose and heading angles from the quaternion, then publishing linear and angular velocities to /cmd_vel

## 2D Map with Clearances Applied


![image](https://github.com/user-attachments/assets/14151b3c-c3fe-4c75-966d-a0b403a5545d)


## 2D Map with Generated Path

![image](https://github.com/user-attachments/assets/7f23e4b0-3b3d-4468-9dac-325c41fc2615)

## 3D Map in Gazebo

![gazebo](https://github.com/user-attachments/assets/44055b91-de16-472b-9eb6-26878d8b54f4)

## 3D Map in Falcon Simulator

![image](https://github.com/user-attachments/assets/a202a682-5a2d-4597-a815-1ee8281674fa)


# Running Code

## Clone the Repository


```sh
git clone https://github.com/khuzema-h/Turtlebot_3_A_star_Path_Finder.git
cd ~/Turtlebot_3_A_star_Path_Finder
```

## Build and Source Workspace

```sh
cd ~/Turtlebot_3_A_star_Path_Finder
colcon build
source install/setup.bash
```
## Run 2D Path Planner

```sh
cd ~/Turtlebot_3_A_star_Path_Finder/src/turtlebot3_project3
python3 astar.py
```
## Suggested Values for User Input:

Robot Clearance : 30

Start Coordinates : 220, 1500, 30

Goal Coordinates : 5000,1500

RPM1 : 5

RPM2 : 10

Sample Outout: 


```sh
Enter Robot Clearance (in millimeters 10 - 40) : 30
Enter start x (in millimeters): 220
Enter start y (in millimeters): 1500
Enter start theta (in degrees): 30
Enter goal x (in millimeters): 5000
Enter goal y (in millimeters): 1500
Enter RPM1 (1 - 10): 5
Enter RPM2 (10 - 20): 10
Generating Path....

```

# Run Gazebo Simulation 

## Build and Source Workspace

```sh
cd ~/Turtlebot_3_A_star_Path_Finder
colcon build
source install/setup.bash
```

## Launch Gazebo Competition World 

```sh
ros2 launch turtlebot3_project3 competition_world.launch.py
```

## Run Trajectory Controller

In a separate terminal run the following command: 

```sh
ros2 run turtlebot3_project3 trajectory_controller.py
```

# Run Falcon Simulation 

Place the `astar_falcon_planner` package in the src directory of the `falcon_turtlebot3_project_ws` workspace and build it

```
cd falcon_turtlebot3_project3_ws/
colcon build
source install/setup.bash
```
To run the simulation
```
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py delta_time:=0.5
```

As we are using the same user input for both falcon and gazebo, only the `delta_time` parameter is set in command line. The waypoints are read from the `waypoints.txt` file. If the user wishes to change the start coordinate, it has to be set in the `AMRPathPlanning.usda` file(line 50 for position and line 51 for orientation). The turtlebot spawns at `(1630, 700, 0)` in the falcon simulatioh. If the start position needs to be `(0.5, 1.0)` meters, this needs to be modified to `(1680, 800, 0)`. 

**Note: Place the waypoints.txt file right inside the `falcon_turtlebot3_project_ws` workspace. This is crucial for the source file to read from the waypoints.txt file**






