# CMakeLists.txt per il pacchetto cooperative_localization
#
# Questo file CMake definisce la configurazione di build per il sistema di
# localizzazione cooperativa centralizzato basato su Extended Kalman Filter.
#
# @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
# @date 2025

cmake_minimum_required(VERSION 3.8) 
project(cooperative_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) 
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall      
    -Wextra    
    -Wpedantic 
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                     
find_package(nav_msgs REQUIRED)                   
find_package(geometry_msgs REQUIRED)              
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include              
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(coop_localization_node
  src/centralized_node.cpp    
  src/ekf_centralized.cpp     
  src/data_loader.cpp         
  src/groundtruth_loader.cpp  )

target_include_directories(coop_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
  $<INSTALL_INTERFACE:include>                            
)

target_compile_options(coop_localization_node PRIVATE
  -march=native  
)

ament_target_dependencies(coop_localization_node
  rclcpp        
  nav_msgs      
  geometry_msgs 
  std_msgs      
)


add_executable(robot_node
  src/robot_node.cpp     
  src/data_loader.cpp    
)

target_include_directories(robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(robot_node PRIVATE
  -march=native
)

ament_target_dependencies(robot_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)


add_executable(test_dataset
  src/test_dataset.cpp    
  src/ekf_centralized.cpp 
  src/data_loader.cpp     
)

target_include_directories(test_dataset PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(test_dataset PRIVATE
  -march=native
)

ament_target_dependencies(test_dataset
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(evaluate_accuracy
  src/evaluate_accuracy.cpp   
  src/ekf_centralized.cpp     
  src/data_loader.cpp         
  src/groundtruth_loader.cpp  
)

target_include_directories(evaluate_accuracy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(evaluate_accuracy PRIVATE
  -march=native
)

ament_target_dependencies(evaluate_accuracy
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  coop_localization_node  
  robot_node              
  test_dataset           
  evaluate_accuracy      
  DESTINATION lib/${PROJECT_NAME}  
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() 
endif()

ament_package()
