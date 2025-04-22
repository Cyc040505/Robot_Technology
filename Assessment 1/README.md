# Catch Turtle All
### Directory Structure
```
Assement 1/  # workstation
├── src/
│   └── catch_turtle_all/
│       ├── catch_turtle_all/  # code files
│       │   ├── __init__.py
│       │   ├── turtle_spawner.py  # task A: generate turtles
│       │   ├── 
│       │   ├── 
│       │   └── 
│       ├── launch/
│       │   └── catch_turtle.launch.py  # launch file
│       ├── resource/
│       ├── package.xml  # dependency file
│       ├── setup.cfg
│       └── setup.py  # build project file
└── README.md  # this file
```
### Function Details
1. **Generate turtles**: Generate a turtle randomly every 3 seconds.
2. 

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