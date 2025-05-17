# ENPM 661 Project 5 : Pursuit and Evasion 
#### Project Members : Adil Qureshi (1220607905), Khuzema Habib(12188776), Sounderya Venugopal(121272423)

##  Project Description

In this project, we implement Pursuit and Evasion in 2 different environments:

- 2D Environment
- Gazebo


We first run the 2D Path Planner to generate a trajectory for the other simulations in 3D, applying clearances where required. The 2D Map was generated with an online map generation tool. 

Link to Folder Containing Path Generation, 2D and Gazebo Simulations can be found here : [Google Drive Folder](https://drive.google.com/drive/folders/1j4QxEiSWiU6yOw_LebiXOuJSLmVj8Trs?usp=sharing)

## Gazebo Trajectory Controller



The controller implements proportional control to follow the waypoints, subscribing to /odom to extract pose and heading angles from the quaternion, then publishing linear and angular velocities to /cmd_vel

## 2D Map 


![maze](https://github.com/user-attachments/assets/4814dff4-bfa3-4ede-99c3-6194a3214493)



## 2D Map with Generated Path

![rrt static](https://github.com/user-attachments/assets/9c870eb4-9d9f-41e7-8c2f-8b1ae7fd4fff)


## 3D Map in Gazebo
![3d map](https://github.com/user-attachments/assets/6342adff-e24d-4070-aacf-762a060a3731)


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
python3 rrt_.py

```
## Suggested Values for User Input:


```sh
Enter start point (x y): 10 10
Enter initial goal point (x y): 150 75
🎯 Reached moving goal at step 55

```
## Dynamic RRT* Path Planner


### Step 18

![1](https://github.com/user-attachments/assets/968a5db2-5c10-4d5d-9209-81995b1c8304)

### Step 28

![2](https://github.com/user-attachments/assets/2c80a9b6-f825-4553-98c2-72b4ce8ef55d)

### Step 38: Goal Reached!

![3](https://github.com/user-attachments/assets/211af0a0-b672-419b-8354-66c7c23b09f5)



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
## Run RRT* Planner Script

In a separate terminal run the following command: 

```sh
ros2 run turtlebot3_gazebo planner.py
```

## Run Turtlebot Evasion Script

In a separate terminal run the following command: 

```sh
ros2 run turtlebot3_gazebo evader_path.py
```


## Run Turtlebot Red Dectection Script

In a separate terminal run the following command: 

```sh
ros2 run turtlebot3_gazebo red_detector.py
```

## Run Pursuer Navigator Script

In a separate terminal run the following command: 

```sh
ros2 run turtlebot3_gazebo navigator.py
```







