cmake_minimum_required (VERSION 3.10)
project (volumetric_drilling_msgs)

find_package (catkin QUIET COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

set (MESSAGES
  Index.msg
  Voxels.msg
  DrillSize.msg
  VolumeInfo.msg
  )

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
else (catkin_FOUND)
  # look for ROS2
  find_package(ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    find_package (rosidl_default_generators REQUIRED)
    find_package (std_msgs REQUIRED)
    find_package (geometry_msgs REQUIRED)
  endif (ament_cmake_FOUND)
endif(catkin_FOUND)

if (ROS_VERSION EQUAL 1)
  add_message_files (
    FILES
    ${MESSAGES}
  )
  generate_messages (
    DEPENDENCIES
    geometry_msgs
    std_msgs
  )
  catkin_package (
    CATKIN_DEPENDS message_runtime
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

else (ROS_VERSION EQUAL 2)

  set (MESSAGES_ROS2 "")
  foreach (_msg ${MESSAGES})
    list (APPEND MESSAGES_ROS2  "msg/${_msg}")
  endforeach ()

  rosidl_generate_interfaces (
    volumetric_drilling_msgs
    ${MESSAGES_ROS2}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)

  ament_package ()
endif ()
