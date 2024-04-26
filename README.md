# RobExplore
Is capable of multirobot exploration of unknown environment in VREP simulation. The robots utilze LIDARs to scan the surrounding environment. Scans are used to create a map of surrounding environment, which allows the robots to avoid obstacles and select the most promissing frontiers for future exploration.

TODO add some image

## Installation & Usage
Install the required packages using:
```bash
pip install -r requirements.txt
```
Download CoppeliaSim Edu from [web](https://www.coppeliarobotics.com/downloads) and extract the archive. The simulator can be than started using:
```bash
./coppeliaSim.sh
```
You can than load the scenes from the `vrep_scenes` directory. For the multirobot exploration use the `locks_multirobot.ttt` scene.

Run the multirobot exploration using:
```bash
python Explorer.py
```

TODO Quickly describe that you have to install VREP and some more python packages to fully use my thing.

## Features
The created approach consists of several components, which are described in the following sections. The components are designed to be modular, so they can be easily replaced or extended. The components are:

### Mapping 
The robots collect the information from the lidar sensor and create a map of the environment. The created map is automatically enlarged when the laser measurements fall out of its boundaries.

### Frontier detection 
Free cells neighbouring with unobserved ones are grouped into free edge components, which are the most promissing for exploration. The precise goals are then created using k-means clustering. Finally, the mutual information can be computed using the presented formulae from a region around the frontier to select the most interesting one. The region is a square with the frontier in the middle, as it is assumed that the robot observes all the cells in case of frontier exploration.

### Path planning 
The A* algorithm finds the shortest path to the selected frontier. First, the obstacles are inflated in the map to ensure that the robot does not collide with them. However, a small area around the robot is guaranteed free; otherwise, the planning could fail. Note that the unobserved cells are not inflated, as such action would prevent the robot from reaching the frontiers. Goals can be selected from the frontier based on the path length or the mutual information.

### Navigation
The robot follows the navigation goal using a simple control method. The robot is commanded to perform either forward or rotational movement. Such clear distinction allows the Navigation to closely follow the planned path using the A* algorithm, as the path consists of straight line segments. Additionally, the robot replans the path to account for unobserved cells transformed into obstacles. New obstacles create more inflated areas; thus, the robot has to replan the path to avoid the newly inflated obstacles. The path is updated if the distance between the first point of the planned path and the robot is greater than a certain threshold. Otherwise, the robot could start oscillating.

### Multirobot exploration 
The environment can be explored using multiple robots. The goals for robots are selected either greedily as the goal with the highest criterium value or using the MinPos strategy. The MinPos strategy ranks goals for each robot according to the number of robots closer to the goal than the robot itself. Then, the robot selects the goal with the lowest rank and minimal criteria among these ranks.

## Note
This is an implementation of semestral project for a Artificial Intelligence course at FEE CTU. 