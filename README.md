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
### Bug 0 Algorithm
```
while not at goal:
    move towards the goal

    if obstacle encountered:
        while unable to move directly towards the goal:
            follow the obstacle’s boundary on the left side
```

![Bug 2 Example](images/Screenshot%20from%202025-07-22%2011-04-14.png)

