# Catch Turtle All
### Directory Structure
```
Assement 1/  # workstation
├── src/
│   └── catch_turtle_all/
│       ├── catch_turtle_all/
│       │   ├── __init__.py
│       │   ├── turtle_spawner.py  # Sub-task A: generate turtles
│       │   ├── master_turtle.py  # Sub-task B: control the master turtle
│       │   ├── turtle_follower.py  # Sub-task C: follow the master turtle
│       │   └── turtle_controller.py  # Coordinate all functions
│       ├── launch/
│       │   └── catch_turtle.launch.py  # launch file
│       ├── resource/
│       ├── test/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── README.md  # this file
```

### Run Project
open the workstation in terminal:
```
cd your/path/to/the/workstation/
```
build the project:
```
colcon build --packages-select catch_turtle_all
source install/setup.bash
```
run the project:
```
ros2 launch catch_turtle_all catch_turtle.launch.py
```