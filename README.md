### Depends on installation

Install serial port library：

    cd serial && mkdir build && cd build
    cmake .. && make
    sudo make install

Install other library：    

    sudo apt update && sudo apt install libserial-dev  ros-humble-imu-tools

### How to use

Place the folder in the ROS workspace(ros_ws/src/imu)， and turn to the workspace folder(ros_ws/):

    colcon build --packages-select imu  
    source install/setup.bash

just connect imu :

    ros2 run imu imu_node --ros-args --param port_name:=/dev/ttyUSB0

view with rviz :

    ros2 launch imu imu_view.launch.py port_name:=/dev/ttyUSB0

