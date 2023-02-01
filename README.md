# Suction Experiment
Experiment to test the performance of the suction cup when cartesian and angular noise is added to its position w.r.t. the center of a sphere. This matters for our gripper design, in order to build better insights of the dimensions used in the gripper.


# Basic Installation

The following steps were performed under Ubuntu 20.04.5 LTS (Focal Fossa)

### Arduino  
1. Install Arduino  
2. Search and install the libraries **rosserial** and **adafruit mprls**  
3. If after compiling there are issues with *cstring*, simply:  
open **msg.h**  
replace `#include <cstring>` with `#include<string.h>`  
replace `std::memcpy()` with `memcpy()` 
     
### ROS
1. Install ROS [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Create WorkSpace  
```console
mkdir -p your_ws/src && cd catkin_ws
```

3. Install **rosserial** package  
```console
cd <your_ws>/src  
git clone https://github.com/ros-drivers/rosserial.git  
cd <your_ws>  
catkin_make
```

4. Install **opencv** package
```console
cd <your_ws>/src 
git clone https://github.com/ros-drivers/video_stream_opencv
cd <your_ws>
catkin_make
```


5. Don't forget to source the workspace   
`gedit ~/.bashrc` and add this line at the end `source ~/<your_ws>/devel/setup.bash` 

5. Install **moveit** package
```console
sudo apt install ros-noetic-moveit
```

# Running
1. Upload code into **arduino board**.
2. Check the port name, and make sure it matches the one written at line 50 of **serial_node.py**.
3. Launch the lab setup in terminal one:
```console
roslaunch apple_proxy pickApp.launch
```
4. Run RosSerial node in terminal two
```console
rosrun rosserial_python serial_node.py
```
4. Run OpenCv streamng in terminal three
```console
roslaunch video_stream_opencv camera.launch video_stream_provider:=/dev/video2
```
5. Run Experiment code in terminal three
```console
python3 suction_experiment.py
 ```

## Tips  
If you want to read a certain sensor/topic from command line:
```console
rostopic echo /gripper/pressure
```
Also, if you want to send a control command from command line:
```console
rosservice call openValve
```
or
```console
rosservice call closeValve
```

Tips for the camera:
https://askubuntu.com/questions/348838/how-to-check-available-webcams-from-the-command-line