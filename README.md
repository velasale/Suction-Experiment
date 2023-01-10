# Suction Experiment

# Basic Installation

The following steps were performed under Ubuntu 20.04.5 LTS (Focal Fossa)

### Arduino  
1. Install Arduino  
2. Search and install libraries **rosserial** and **adafruit mprls**  
3. If after compiling there are issues with <cstring>, simply:  
open **msg.h**  
replace `#include <cstring>` with `#include<string.h>`
replace `std::memcpy()` with `memcpy()` 
     
### ROS
 
