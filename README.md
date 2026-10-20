# Running Instructions
Prem Sethumadhavan and Trevor McDonald\
October 20th, 2026

Writeup Link: 
https://docs.google.com/document/d/1GHLFWkFF0jOyYiEdTT2L7CavwaV04O-_d_LY-1KIvWg/edit?usp=sharing

Running the particle filter should be relatively simple. There are some pre-requisite files you must have.
- Make sure pf.py, occupancy_field.py, helper_functions.py, and angle_helpers.py are in the robot_localization folder.
- Have mac_1st_floor_final.yaml installed somewhere on your computer.
- Have either the macfirst_floor_take_1 or macfirst_floor_take_2 bag installed.
- Have the turtlebot_bag_files.rviz setup for rviz

To run the replay

1. Use the command
```
ros2 launch robot_localization test_pf.py map_yaml:=path-to-your-yaml-file
```
If your file system is set up the same way as ours, the path is (But with your user's name)
```
/home/trevor/ros2_ws/src/robot_localization/maps/mac_1st_floor_final.yaml
```
2. Open another terminal, and open rviz using the turtlebot configuration
```
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz
```
3. Lastly, run the ros bag to see the particle filter in action.
```
ros2 bag play path-to-your-bag-file --clock 
```
Use one of the following paths if you're file system is set up the same way as ours (But with your user's name)
```
/home/trevor/ros2_ws/src/robot_localization/bags/macfirst_floor_take_1
/home/trevor/ros2_ws/src/robot_localization/bags/macfirst_floor_take_2