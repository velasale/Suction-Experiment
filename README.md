# Suction Experiment

# Basic Installation

The following steps were performed under Ubuntu 20.04.5 LTS (Focal Fossa)

### Arduino  
1. Install Arduino  
2. Search and install libraries **rosserial** and **adafruit mprls**  
3. If after compiling there are issues with *cstring*, simply:  
open **msg.h**  
replace `#include <cstring>` with `#include<string.h>`  
replace `std::memcpy()` with `memcpy()` 
     
### ROS
1. Install ROS [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Create WorkSpace  
``mkdir -p your_ws/src && cd catkin_ws``
3. Install **rosserial** package  
``cd <your_ws>/src``  
``git clone https://github.com/ros-drivers/rosserial.git``  
``cd <your_ws>``  
``catkin_make``
4. Don't forget to source the workspace   
`gedit ~/.bashrc` and add this line at the end `source ~/<your_ws>/devel/setup.bash` 

# Running
1. Run `roscore` in terminal one
2. Run `rosrun rosserial_python serial_node.py` in terminal two
3. Run `rostopic list` in another terminal
   
