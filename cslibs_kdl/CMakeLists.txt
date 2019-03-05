cmake_minimum_required(VERSION 2.8.3)
project(cslibs_kdl)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    kdl_parser
    roscpp
    rostest
    tf
    tf_conversions
    trac_ik_lib
    cslibs_kdl_data
    )

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED VERSION 3.3.5)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES cslibs_kdl
      CATKIN_DEPENDS kdl_parser roscpp rostest tf tf_conversions trac_ik_lib cslibs_kdl_data
      DEPENDS Boost orocos_kdl Eigen3
    )

###########
## Build ##
###########


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)

add_library(cslibs_kdl
    src/external_forces.cpp
    src/kdl_joint_state_data.cpp
    src/dynamic_model.cpp
    src/kinematic_model.cpp
    src/yaml_to_kdl_tranform.cpp
    src/residual_vector.cpp
    src/kdl_conversion.cpp
    src/residual_vector_kalman.cpp
    ${ALL_HEADERS}
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(cslibs_kdl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(cslibs_kdl_tf_static_transforms_conversion src/cslibs_kdl_tf_static_tranforms_conversion.cpp)
add_executable(cslibs_kdl_test test/test_cslibs_kdl.cpp)
add_executable(external_torque_observer src/node/external_torque_observer.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(cslibs_kdl_tf_static_transforms_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cslibs_kdl)
add_dependencies(cslibs_kdl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GTEST_LIBRARIES} cslibs_kdl)
add_dependencies(external_torque_observer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cslibs_kdl)
## Specify libraries to link a library or executable target against
target_link_libraries(cslibs_kdl
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    yaml-cpp
    )

target_link_libraries(cslibs_kdl_tf_static_transforms_conversion
    ${catkin_LIBRARIES}
    cslibs_kdl
    )

target_link_libraries(cslibs_kdl_test
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${GTEST_LIBRARIES}
    cslibs_kdl
    )

target_link_libraries(external_torque_observer
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    cslibs_kdl
    )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cslibs_kdl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
