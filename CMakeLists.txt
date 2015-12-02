cmake_minimum_required(VERSION 2.8.3)
project(mwm)

find_package(catkin REQUIRED COMPONENTS
    geolib2
    rgbd
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

add_library(mwm
    include/mwm/world_model.h
    include/mwm/projection_matrix.h
    include/mwm/rendering.h
    include/mwm/triangle.h

    src/world_model.cpp
    src/projection_matrix.cpp
    src/rendering.cpp
)
target_link_libraries(mwm ${catkin_LIBRARIES})

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

add_library(mwm_ros
    include/mwm/ros/image_buffer.h
    src/ros/image_buffer.cpp

)
target_link_libraries(mwm_ros mwm ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_mwm_amigo test/amigo.cpp)
target_link_libraries(test_mwm_amigo mwm_ros)


