cmake_minimum_required(VERSION 2.8.3)

project(han_auto_test)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs message_generation std_msgs)

add_message_files(FILES MsgTutorial.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
#INCLUDE_DIRS include
LIBRARIES han_auto_test
CATKIN_DEPENDS roscpp sensor_msgs std_msgs
DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subscriber2 src/subscriber2.cpp)
target_link_libraries(subscriber2 ${catkin_LIBRARIES})
add_dependencies(subscriber2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subscriber3 src/subscriber3.cpp)
target_link_libraries(subscriber3 ${catkin_LIBRARIES})
add_dependencies(subscriber3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

