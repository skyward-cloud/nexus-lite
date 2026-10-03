cmake_minimum_required(VERSION 3.10)
project(px4_sitl_gz_launch)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

function(find_gz_pkg out_prefix base_name)
  foreach(ver IN ITEMS 14 13 12 11 10 9)
    pkg_check_modules(${out_prefix} QUIET ${base_name}${ver})
    if(${out_prefix}_FOUND)
      message(STATUS "Found ${base_name}${ver} via pkg-config")
      return()
    endif()
  endforeach()
  message(FATAL_ERROR "Could not find ${base_name} (tried versions 9-14)")
endfunction()

find_gz_pkg(GZ_TRANSPORT gz-transport)
find_gz_pkg(GZ_MSGS gz-msgs)

add_executable(gz_transport_listener src/gz_transport.cpp)
target_include_directories(gz_transport_listener PRIVATE
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)
target_link_directories(gz_transport_listener PRIVATE
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)
target_link_libraries(gz_transport_listener
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)
target_compile_options(gz_transport_listener PRIVATE
  ${GZ_TRANSPORT_CFLAGS_OTHER}
  ${GZ_MSGS_CFLAGS_OTHER}
)
ament_target_dependencies(gz_transport_listener
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS gz_transport_listener
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Workspace simulation assets → share/<pkg>/{models,worlds} for get_package_share_directory.
set(_NEXUS_SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../simulation")
if(EXISTS "${_NEXUS_SIM_DIR}/models")
  install(DIRECTORY "${_NEXUS_SIM_DIR}/models"
    DESTINATION share/${PROJECT_NAME}
  )
endif()
if(EXISTS "${_NEXUS_SIM_DIR}/worlds")
  install(DIRECTORY "${_NEXUS_SIM_DIR}/worlds"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
