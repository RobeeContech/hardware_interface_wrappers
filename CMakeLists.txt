cmake_minimum_required(VERSION 3.8)
project(hardware_interface_wrappers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    transmission_interface
    transmission_manager
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/transmission_wrapper.cpp
  test/testable_joint_system_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_INTERFACE_WRAPPERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_wrappers_ros2_control_plugins.xml)




# INSTALL
install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
    urdf
    test
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake)
  foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach()

  ament_add_gmock(
    test_testable_joint_system_interface
    test/test_testable_joint_system_interface.cpp
  )
  target_include_directories(test_testable_joint_system_interface PRIVATE include)
  ament_target_dependencies(test_testable_joint_system_interface hardware_interface)

   ament_add_gmock(
     test_transmission_wrapper
     test/test_transmission_wrapper.cpp
   )
   target_include_directories(test_transmission_wrapper PRIVATE include)
   target_link_libraries(test_transmission_wrapper
     hardware_interface_wrappers
    )

    ament_lint_auto_find_test_dependencies()
    ament_add_gmock(test_velocity_state_transmission test/test_velocity_state_transmission.cpp)
    target_include_directories(test_velocity_state_transmission PRIVATE include)
    target_link_libraries(test_velocity_state_transmission ${PROJECT_NAME})
    ament_target_dependencies(test_velocity_state_transmission ${THIS_PACKAGE_INCLUDE_DEPENDS})
    add_launch_test(test/test_transmission_wrapper_robot1.py)
    add_launch_test(test/test_transmission_wrapper_robot2.py)
    add_launch_test(test/test_testable_joint_system_robot1.py)
    add_launch_test(test/test_testable_joint_system_robot2.py)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
  transmission_manager
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
