## Setting up workspace with ROS + Python 3 


change first line to cmake_minimum_required(VERSION 2.8.3) in geometry/geometry/CMakeLists.txt

from geometry/tf/CMakeLists.txt comment the following
1. ``` 
    if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    endif()
    ```
2. ```
   add_executable(transform_listener_unittest test/transform_listener_unittest.cpp)
   target_link_libraries(transform_listener_unittest ${PROJECT_NAME} ${GTEST_LIBRARIES})
   add_rostest(test/transform_listener_unittest.launch)
   ```

```
git clone --single-branch --branch melodic-devel https://github.com/ros/geometry
git clone --single-branch --branch melodic-devel https://github.com/ros/robot_state_publisher.git 
git clone --single-branch --branch melodic-devel https://github.com/ros/kdl_parser.git
git clone https://github.com/ros/angles.git
```

## Setting up ROS + Networking