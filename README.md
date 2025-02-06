# Package for detecting solar panel pose using RS-BP scanner on Solar Cleaning Robot

## requirement
1. build & install rs_driver (https://github.com/RoboSense-LiDAR/rs_driver/tree/main)
   
```
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
git clone https://github.com/RoboSense-LiDAR/rs_driver/tree/main
cd rs_driver
```
if using clang instead of gcc/g++, change the following line in the CMakeLists.txt file
```
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
```
to
```
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
```
then, if want to compile demo and tools, 
```
mkdir build 
cd build
cmake -DCOMPILE_DEMOS=ON -DCOMPILE_TOOLS=ON .. 
make -j4
sudo make install
```
## visualize a recorded pcap file
```
./build/tool/rs_driver_viewer -type RSBP -pcap /PATH_TO_PCAP.pcap 
```
sensor types are defined in file: rs_driver/src/rs_driver/driver/driver_param.hpp file
## run demo
compile the code
```
mkdir build 
cd build
cmake ..
make
```
run demo
```
cd build
./handle_pcap [-pcap /PATH_TO_PCAP.pcap] [-type RSBP]
```
type is by default to RSBP. pcap file can also be set in the handle_pcap.cpp file.
## TODO
(1) LINE2_x is not exactly 0, make it 0.
(2) transfer params with yaml file.