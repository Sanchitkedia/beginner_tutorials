# ROS2 Humble Hawksbill Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
This repository contains the ROS2 tutorial implementation for Humble Hawksbill.
The programs are adapted from the ROS2 tutorials to meet the course requirements ENPM808X - Software Development for Robotics.

## Personnel
- Sanchit Kedia (UID: 119159620) 

# Dependencies
- ROS2 (Humble Hawksbill Source Build)
- Ubuntu 20.04 LTS
- Terminator (Linux terminal emulator)

# Build Package
```
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
cd <Your ROS2 workspace src folder>
git clone https://github.com/Sanchitkedia/beginner_tutorials.git
cd ..
rosdep install -i --from-path src --rosdistro humble -y #Check for missing dependencies
colcon build --packages-select beginner_tutorials
```

## Run Package
### [1] In a new terminal (Launch Talker Node with server to modify the published string)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 run beginner_tutorials talker
```
### [2] In a another terminal (Launch Listener Node)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 run beginner_tutorials listener
```

### [3] In a another terminal (Launch Call Service)
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 service call /string_change beginner_tutorials/srv/StringChange "{input: '<The String You Want to Publish>'}"
```

### [4] Launch all nodes with arguments using launch file
```
cd <Your ROS2 workspace>
source ~/ros2_humble/install/local_setup.bash  # Source your ROS2 Installation (Path may vary)
source . install/setup.bash #Source the setup files
ros2 launch beginner_tutorials week_10.launch Publisher_Frequency:=0.5 # Launch file launches listener node in a seperate terminator window (Need terminator to be installed)
```

## Cpplint

```
pip3 install cpplint # If not already installed
cd <Your ROS2 workspace>/src/beginner_tutorials/src
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order publisher_member_function.cpp > ../results/publisher_member_function_cpp.txt
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order subscriber_member_function.cpp > ../results/subscriber_member_function_cpp.txt
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
