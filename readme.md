# ROS2 Humble Hawksbill Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
This repository contains the ROS2 tutorial implementation for Humble Hawksbill.
The programs are adapted from the ROS2 tutorials to meet the course requirements ENPM808X - Software Development for Robotics.
The repository has mutiple branches for each assignment from the course ENPM808X.
The main branch contains the the latest version of the code which has been merged from all the other branches.
[1] [ROS2 Publisher and Subscriber](https://umd.instructure.com/courses/1336069/assignments/6179366) (branch - ros_pub_sub)
[2] [ROS2 Services,Logging and Launch](https://umd.instructure.com/courses/1336069/assignments/6182236) (branch - WEEK10_HW)
[3] [ROS2 tf2, unit testing and bag files](https://umd.instructure.com/courses/1336069/assignments/6188169) (branch - WEEK11_HW)

## Personnel
- Sanchit Kedia (UID: 119159620) 

# Dependencies
- ROS2 (Humble Hawksbill Source Build)
- Ubuntu 20.04 LTS

# Build Package
```
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
cd <Your ROS2 workspace src folder>
git clone -b Week10_HW https://github.com/Sanchitkedia/beginner_tutorials.git
cd ..
rosdep install -i --from-path src --rosdistro humble -y #Check for missing dependencies
colcon build --packages-select beginner_tutorials
```

## Run Package
### [1] Launch all nodes with arguments using launch file
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 launch beginner_tutorials pub_sub.launch Publisher_Frequency:=<Any value for frequency in Hz (Double)> # Launch file launches listener node in a seperate terminal
```

### [2] In a another terminal (Call Service if you want to change the default string)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 service call /string_change beginner_tutorials/srv/StringChange "{input: '<The String You Want to Publish>'}"
```

### [3] In a another terminal (Check the broadcasted tf2 frames)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 run tf2_ros tf2_echo world talk
```

### [4] In a another terminal (Save the tf2 frames to a pdf file)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 run tf2_tools view_frames #Save the pdf file in the same directory
```
## Run Unit Tests
### [1] launch Unit Testing node
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 launch beginner_tutorials test.launch #Launches the unit test
# Press Ctrl+C to exit the test after it passes as the launched talker node will keep publishing
```
## Run rosbag to record and play the data
### [1] In a another terminal (Record the bag file)
You can change the name of the bag file by providing the name as an argument to the launch file by using bag_name:=<Your Bag File Name> (Default name is "pub_sub_bagfile")
The bag file is saved in the bagfiles directory in the package
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 launch beginner_tutorials pub_sub.launch record:=true #Launches the talker node and records the bag file
# Wait for 15 seconds and press Ctrl+C to exit and stop the recording.
```
### [2] In a another terminal (View the details about the bag file)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 bag info src/beginner_tutorials/bagfiles/<Your Bag File Name> #View the details about the bag file
```

### [3] Play the bag file
#### [3.1] In a another terminal (Launch the listener node)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 launch beginner_tutorials pub_sub.launch #Launches the listener node
```
#### [3.2] In a another terminal (Play the bag file)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 bag play src/beginner_tutorials/bagfiles/<Your Bag File Name> #Play the bag file
```

## Cpplint
```
pip3 install cpplint # If not already installed
cd <Your ROS2 workspace>/src/beginner_tutorials/src
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order publisher_member_function.cpp > ../results/publisher_member_function_cpp.txt
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order subscriber_member_function.cpp > ../results/subscriber_member_function_cpp.txt
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order test_pub.cpp > ../results/test_pub_cpp.txt
```

## Cppcheck
```
sudo apt-get install cppcheck # If not already installed
cd <Your ROS2 workspace>/src/beginner_tutorials/src
cppcheck --enable=all --std=c++17 --force --suppress=missingIncludeSystem publisher_member_function.cpp subscriber_member_function.cpp > ../results/cppcheck.txt
```

## Reference

[1] [ROS2 Humble Hawksbill Tutorials](http://docs.ros.org/en/humble/Tutorials.html)

## License

MIT License

Copyright (c) 2022 Sanchit Kedia.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
