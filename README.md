# group4-20fa-ece498sma

Final Project to Implement Adaptive Cruise Control for the GEM Vehicle using ROS and Gazebo for simulation.


## Recording data - Highbay

    rosbag record /lidar1/scan /lidar1/velodyne_points /mako_1/mako_1/image_raw /novatel/imu -e "/front_radar/(.*)" -O inst5


## Getting Started

Navigate to the root of the directory:

    catkin_make
    source devel/setup.bash

Start roscore on another terminal by typing:

    roscore

To launch the default gazebo and rviz:

    roslaunch mp5 mp5.launch
    
On another terminal:

All the python files are located in the following directory:

    cd group4-20fa-ece498sma/src/mp5/src

For Simulation:

    python set_pos.py --x 5.39069 --y -100.765
    python main.py gem run
    python main.py gem stationary (To keep the car still)


For Highbay: (Navigate to the folders with the rostopics if looking at rosbags, or do nothing if live)

    rosbag play -l <filename>.bag

On another terminal:

    python main.py highbay run

On another terminal open rviz to visualize the different rostopics:

    rviz



NOTE: Every new terminal might need you to source from the project root directory:

    source devel/setup.bash










## Resources / Links

https://uofi.app.box.com/s/srwcxx2mkwgmvf3nixk1jsh72qofeeky/file/715137148320 - GEM Car Manual

http://luthuli.cs.uiuc.edu/~daf/courses/MAAV-2020/598-2020-vehicle-resources.html - Vehicle Resources

http://luthuli.cs.uiuc.edu/~daf/courses/MAAV-2020/598-2020-lecture-materials.html -  ROS Resources



## Simulation Information
____

Important Files: 

group4-20fa-ece498sma/src/mp5/launch/mp5.launch - Launch file 


External Packages:

https://github.com/swri-robotics/novatel_gps_driver - GPS Driver


# How-To Common Issues:

How do I add a new package? (ie: GPS)

Copy over the package (list of packages) to the /src directory, and in the project root run catkin_make
source devel/setup.bash


