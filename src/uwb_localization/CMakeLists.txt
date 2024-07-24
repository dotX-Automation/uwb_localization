cmake_minimum_required(VERSION 3.8)
project(uwb_localization)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_node REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(params_manager REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/uwb_localization/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure uwb_localization Node source files
file(GLOB UWB_LOCALIZATION_SRCS
RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
"src/uwb_localization/*.cpp")
list(APPEND UWB_LOCALIZATION_SRCS init_parameters.cpp)

add_library(uwb_localization SHARED ${UWB_LOCALIZATION_SRCS})
target_compile_definitions(uwb_localization PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(uwb_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(uwb_localization
  Ceres
  Eigen3::Eigen
  Threads::Threads)
ament_target_dependencies(uwb_localization
  dua_app_management
  dua_interfaces
  dua_node
  geometry_msgs
  params_manager
  rcl_interfaces
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

# uwb_localization standalone application
add_executable(uwb_localization_app
  src/uwb_localization_app.cpp)
target_include_directories(uwb_localization_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uwb_localization_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(uwb_localization_app uwb_localization)
ament_target_dependencies(uwb_localization_app
  dua_app_management
  rclcpp)

# uwb_localization app
install(TARGETS uwb_localization
  DESTINATION lib/${PROJECT_NAME})

# Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()