#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2021, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev:  $

cmake_minimum_required (VERSION 3.10)
project (volumetric_drilling)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

set (ROS1_depends ambf_server volumetric_drilling_msgs)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  add_definitions (-DROS1)
  set (ROS_VERSION 1)
  include_directories (${catkin_INCLUDE_DIRS})
  
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    add_definitions (-DROS2)
    set (ROS_VERSION 2)
    set (ROS2_depends rclcpp geometry_msgs ambf_msgs volumetric_drilling_msgs ambf_server)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()
    link_directories (${ament_cmake_LIBRARY_DIRS})

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

include_directories (
  ${AMBF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

link_directories (${AMBF_LIBRARY_DIRS})

add_definitions (${AMBF_DEFINITIONS})
  
add_library(volumetric_drilling
  SHARED
  volumetric_drilling/common.h
  volumetric_drilling/volumetric_drilling.cpp
  volumetric_drilling/volumetric_drilling.h
  volumetric_drilling/ros_interface.cpp
  volumetric_drilling/ros_interface.h
  volumetric_drilling/joystick.cpp
  volumetric_drilling/joystick.h
  volumetric_drilling/footpedal.cpp
  volumetric_drilling/footpedal.h
  volumetric_drilling/wave_generator.cpp
  volumetric_drilling/wave_generator.h
  volumetric_drilling/drill_manager.cpp
  volumetric_drilling/drill_manager.h
  volumetric_drilling/camera_panel_manager.h
  volumetric_drilling/camera_panel_manager.cpp
  volumetric_drilling/gaze_marker_controller.h
  volumetric_drilling/gaze_marker_controller.cpp)

target_link_libraries (
  volumetric_drilling
  ${Boost_LIBRARIES}
  ${AMBF_LIBRARIES})
set_property (TARGET volumetric_drilling PROPERTY POSITION_INDEPENDENT_CODE TRUE)

add_library (camera_hmd
  camera_hmd/hmd.cpp
  camera_hmd/hmd.h)
target_link_libraries (camera_hmd ${AMBF_LIBRARIES})
set_property (TARGET camera_hmd PROPERTY POSITION_INDEPENDENT_CODE TRUE)


if (ROS_VERSION EQUAL 1)
  add_dependencies (volumetric_drilling ${catkin_EXPORTED_TARGETS})
  target_link_libraries (volumetric_drilling ${catkin_LIBRARIES})

else ()

  ament_target_dependencies (
    volumetric_drilling
    ${ROS2_depends})

  install (
    DIRECTORY volumetric_drilling
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    )

  install (
    TARGETS volumetric_drilling
    EXPORT volumetric_drilling
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

  ament_export_targets (volumetric_drilling HAS_LIBRARY_TARGET)
  ament_export_dependencies (${ROS2_depends})
  ament_package ()

  install (
    TARGETS camera_hmd
    EXPORT camera_hmd
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

endif ()
