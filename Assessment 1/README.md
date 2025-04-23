# Catch Turtle All
### Directory Structure
```
Assement 1/  # workstation
├── src/
│   └── catch_turtle_all/
│       ├── catch_turtle_all/  # code files
│       │   ├── __init__.py
│       │   ├── turtle_spawner.py  # task A: generate turtles (Avoid overlap)
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
*Task A: turtle_spawner.py*
1. **Avoid Overlap**: Find the safe area by the spiral search algorithm.
2. **Generate turtles**: Generate a turtle randomly every 3 seconds in the safe area.
3. **Subscribe turtles**: Subscribe and save the locations of all turtles for integrating subsequent algorithms.
4. **Adaptive Timer**: Adaptively change the time interval for generating turtles according to the result of generating turtles.

*Task B*

*Task C*

### Run Project
Open the workstation in terminal:
```
cd your/path/to/the/workstation/
```
Build the project:
```
colcon build --packages-select catch_turtle_all
source install/setup.bash
```
Run the project:
```
ros2 launch catch_turtle_all catch_turtle.launch.py
```
If you find the project running is inconsistent with the expectation of the updated version, please turn to the workstation.
Delete packages "build" and "install", then rebuild the project.
