# TurtleBot3 Bug and Potential Field Algorithm (ROS 2)

A custom ROS 2 package for TurtleBot3 that implements autonomous navigation using **Bug Algorithms** (Bug0, Bug1, Bug2) and **Potential Field Obstacle Avoidance** in custom Gazebo worlds with custom models.

This package and documentation are inspired by concepts from the textbook *Lectures on Robotic Planning and Kinematics* by Francesco Bullo and Stephen L. Smith.

---

### Clone the bug2_nav package

```bash
# Navigate to the TurtleBot3 workspace's src folder
cd ~/turtlebot3_ws/src

# Clone this repository
git clone https://github.com/Phumint/bug2_nav.git

# Build the package
cd ~/turtlebot3_ws
colcon build --packages-select bug2_nav

# Source your workspace
source install/setup.bash
```
### Circumnavigation Algorithm (Wall - Following)
You can run the simulation for the circumvaigation algorithm with the following commands:

1. **In one terminal, launch the world:**

```bash
ros2 launch bug2_nav building_1.launch.py
```
2. **In another terminal, run the Bug 0 node:**
```bash
ros2 run bug2_nav right_wall_following
```
The algorithm flow is as follows:

```
state ← FORWARD

loop:
    read laser scan data
    front_dist ← min(front region)
    right_wall_dist ← average of right-front and right-mid regions

    if state = FORWARD:
        if front_dist < threshold:
            state ← TURN_TO_WALL
        else if in open space:
            state ← SEARCH_FOR_WALL
        else:
            move forward

    else if state = SEARCH_FOR_WALL:
        slowly move and turn right
        if wall detected on right:
            state ← TURN_TO_WALL
        else if obstacle in front:
            state ← TURN_TO_WALL

    else if state = TURN_TO_WALL:
        turn left
        if front is clear and wall on right:
            state ← FOLLOW_WALL
        else if front blocked:
            continue turning
        else:
            state ← SEARCH_FOR_WALL

    else if state = FOLLOW_WALL:
        if front_dist < threshold:
            state ← TURN_TO_WALL
        else if wall lost (right too far):
            state ← TURN_CORNER
        else:
            error ← desired_wall_dist - right_wall_dist
            angular_z ← kp × error
            move forward with angular_z

    else if state = TURN_CORNER:
        slowly move and turn right
        if wall re-detected and front is clear:
            state ← FOLLOW_WALL
        else if front blocked:
            state ← TURN_TO_WALL
        else if in open space:
            state ← SEARCH_FOR_WALL

```

### Bug 0 Algorithm
```
while not at goal:
    move towards the goal

    if obstacle encountered:
        while unable to move directly towards the goal:
            follow the obstacle’s boundary on the left side
```
![Bug 2 Example](images/Screenshot%20from%202025-07-22%2011-04-14.png)

You can run the example in the image above with the following commands:

1. **In one terminal, launch the world:**

```bash
ros2 launch bug2_nav building_0.launch.py
```
2. **In another terminal, run the Bug 0 node:**
```bash
ros2 run bug2_nav bug0_building_0
```

