# ENPM 661 Project 5 : Pursuit and Evasion 
#### Project Members : Adil Qureshi (1220607905), Khuzema Habib(12188776), Sounderya Venugopal(121272423)

##  Project Description

In this project, we implement Pursuit and Evasion in 2 different environments:

- 2D Environment
- Gazebo


We first run the 2D Path Planner to generate a trajectory for the other simulations in 3D, applying clearances where required. The 2D Map was generated with an online map generation tool. 

Link to Folder Containing Path Generation, 2D and Gazebo Simulations can be found here : 

## Gazebo Trajectory Controller



The controller implements proportional control to follow the waypoints, subscribing to /odom to extract pose and heading angles from the quaternion, then publishing linear and angular velocities to /cmd_vel

## 2D Map with Clearances Applied





## 2D Map with Generated Path



## 3D Map in Gazebo


# Running Code

## Create a new Workspace and Clone the Repository


```sh
mkdir -p chase_ws/src
cd ~/chase_ws/src
git clone https://github.com/khuzema-h/pursuit_evasion.git
```

## Build and Source Workspace

```sh
cd ~/chase_ws
colcon build
source install/setup.bash
```
## Run 2D Path Planner

```sh

```
## Suggested Values for User Input:


```sh


```

# Run Gazebo Simulation 

## Build and Source Workspace

```sh
cd ~/chase_ws
colcon build
source install/setup.bash
```

## Launch Gazebo Maze World with Pursuer and Evader

```sh
ros2 launch turtlebot3_gazebo maze_world.launch.py 

```

## Run Turtlebot Pursuit Evasion Script

In a separate terminal run the following command: 

```sh

```







