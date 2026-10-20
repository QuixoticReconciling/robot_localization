# Bag File Notes
Prem Sethumadhavan and Trevor McDonald\
October 20th, 2026

The files named our_particle_filter_mac_take_1 and our_particle_filter_mac_take_2 are the bag files demonstrating our final particle filters runs.

The take one particle filter test demonstrates that our filter converges relativly quickly and confidently as the particles converge to a relatively small area fast and also stays confiden tthrough the turn the robot takes during its path.

The take 2 particle filter tkes much longer to converge than the first and is noticably less cofident thoughout and doesn't have 100 percent accuracy when it comes to teh lidar lining up with the map. 

To run the files and validate our filter 

1. Open another terminal, and open rviz using the turtlebot configuration

rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz

2. Run the ros bag to see the particle filter in action.

ros2 bag play path-to-bag-file --clock 
