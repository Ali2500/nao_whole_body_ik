cmake_minimum_required(VERSION 2.8.3)
project(nao_whole_body_ik)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roslaunch
  tf
)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core moveit_ros_planning
  DEPENDS Boost HDF5 flann GTest
)

## DEPENDENCIES
##------------------------------------------------------------------------------
## Boost
##------------------------------------------------------------------------------
find_package(Boost REQUIRED)
list(APPEND INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
list(APPEND LIBRARIES    ${Boost_LIBRARIES})

##------------------------------------------------------------------------------
## HDF5
##------------------------------------------------------------------------------
find_package(HDF5 REQUIRED)
list(APPEND INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})
list(APPEND LIBRARY_DIRS ${HDF5_LIBRARY_DIRS})
list(APPEND LIBRARIES    ${HDF5_LIBRARIES})

##------------------------------------------------------------------------------
## GTest
##------------------------------------------------------------------------------
find_package(HDF5 REQUIRED)
list(APPEND INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
list(APPEND LIBRARY_DIRS ${GTEST_LIBRARY_DIRS})
list(APPEND LIBRARIES    ${GTEST_LIBRARIES})

##------------------------------------------------------------------------------
## FLANN
##------------------------------------------------------------------------------
find_package( PkgConfig REQUIRED)
pkg_check_modules( flann REQUIRED flann )

link_directories(${PROJECT_SOURCE_DIR}/include ${LIBRARY_DIRS})
include_directories(include test ${catkin_INCLUDE_DIRS} ${INCLUDE_DIRS})

FILE(GLOB_RECURSE SOURCE_FILES
     src/*.cpp
     include/*.hpp
     include/*.h
     test/*.h
     test/*.cpp)

# Remove cpp files with main()
list(REMOVE_ITEM SOURCE_FILES src/database_generator_node.cpp
                              src/test_node.cpp
                              test/main_gtest.cpp
                              test/run.test)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(database_generator src/database_generator_node.cpp)
target_link_libraries(database_generator ${PROJECT_NAME})

add_executable(example_node example_main.cpp)
target_link_libraries(example_node ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(nao_whole_body_ik_test launch/run.test
                                           test/main_gtest.cpp)
  target_link_libraries(nao_whole_body_ik_test ${PROJECT_NAME})
endif()


##=============================================================================
## Install
##=============================================================================
install(PROGRAMS
    ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/database_generator
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

