//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <nnaguru1@jh.edu>
    \author    Nimesh Nagururu
    \author    Adnan Munawar
*/
//==============================================================================
#include "ros_interface.h"
#include <ambf_server/RosComBase.h>

using namespace std;

DrillingPublisher::DrillingPublisher(const string & a_namespace, const string & a_plugin){
    init(a_namespace, a_plugin);
}

DrillingPublisher::~DrillingPublisher(){
    ambf_ral::publisher_shutdown(m_voxelsRemovalPubPtr);
    ambf_ral::publisher_shutdown(m_drillSizePubPtr);
    ambf_ral::publisher_shutdown(m_volumeInfoPubPtr);
}

void DrillingPublisher::init(const string & a_namespace, const string & a_plugin){
    m_rosNode = afROSNode::getNode(a_plugin);

    ambf_ral::create_publisher<AMBF_RAL_MSG(volumetric_drilling_msgs, Voxels)>
      (m_voxelsRemovalPubPtr,
       m_rosNode,
       a_namespace + "/" + a_plugin + "/voxels_removed",
       1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(volumetric_drilling_msgs, DrillSize)>
      (m_drillSizePubPtr,
       m_rosNode,
       a_namespace + "/" + a_plugin + "/drill_size",
       1, true);
    ambf_ral::create_publisher<AMBF_RAL_MSG(volumetric_drilling_msgs, VolumeInfo)>
      (m_volumeInfoPubPtr,
       m_rosNode,
       a_namespace + "/" + a_plugin + "/volume_info",
       1, true);
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, WrenchStamped)>
      (m_forcefeedbackPubPtr,
       m_rosNode,
       a_namespace + "/" + a_plugin + "/drill_force_feedback",
       1, true);
}

void DrillingPublisher::publishDrillSize(int burrSize, double time){
    m_drill_size_msg.header.stamp = ambf_ral::time_from_seconds(time);
    m_drill_size_msg.size.data = burrSize;

    m_drillSizePubPtr->publish(m_drill_size_msg);
}

void DrillingPublisher::setVolumeInfo(cTransform &pose, cVector3d& dimensions, cVector3d& voxel_count)
{

    cQuaternion quat;
    quat.fromRotMat(pose.getLocalRot());

    m_volume_info_msg.pose.position.x = pose.getLocalPos().x();
    m_volume_info_msg.pose.position.y = pose.getLocalPos().y();
    m_volume_info_msg.pose.position.z = pose.getLocalPos().z();

    m_volume_info_msg.pose.orientation.z = quat.z;
    m_volume_info_msg.pose.orientation.x = quat.x;
    m_volume_info_msg.pose.orientation.y = quat.y;
    m_volume_info_msg.pose.orientation.w = quat.w;

    m_volume_info_msg.dimensions.resize(3);
    m_volume_info_msg.dimensions[0] = dimensions.x();
    m_volume_info_msg.dimensions[1] = dimensions.y();
    m_volume_info_msg.dimensions[2] = dimensions.z();

    m_volume_info_msg.voxel_count.resize(3);
    m_volume_info_msg.voxel_count[0] = voxel_count.x();
    m_volume_info_msg.voxel_count[1] = voxel_count.y();
    m_volume_info_msg.voxel_count[2] = voxel_count.z();
}

void DrillingPublisher::publishVolumeInfo(double time)
{
    m_volume_info_msg.header.stamp = ambf_ral::time_from_seconds(time);
    m_volumeInfoPubPtr->publish(m_volume_info_msg);
}

void DrillingPublisher::appendToVoxelMsg(cVector3d &index, cColorf &color)
{
    AMBF_RAL_MSG(volumetric_drilling_msgs, Index) idx;
    idx.x = index.x(); idx.y = index.y(); idx.z = index.z();
    m_voxel_msg.indices.push_back(idx);
    AMBF_RAL_MSG(std_msgs, ColorRGBA) col;
    col.r = color.getR(); col.g = color.getG(); col.b = color.getB(); col.a = color.getA();
    m_voxel_msg.colors.push_back(col);
}

void DrillingPublisher::clearVoxelMsg()
{
    // Clear the message for next itr.
    m_voxel_msg.indices.clear();
    m_voxel_msg.colors.clear();
}

void DrillingPublisher::publishVoxelMsg(double time)
{
    m_voxel_msg.header.stamp = ambf_ral::time_from_seconds(time);
    m_voxelsRemovalPubPtr->publish(m_voxel_msg);
}

void DrillingPublisher::publishForceFeedback(cVector3d& force, cVector3d& moment, double time)
{
    m_force_feedback_msg.header.stamp = ambf_ral::time_from_seconds(time);
    m_force_feedback_msg.wrench.force.x = force.x();
    m_force_feedback_msg.wrench.force.y = force.y();
    m_force_feedback_msg.wrench.force.z = force.z();

    m_force_feedback_msg.wrench.torque.x = moment.x();
    m_force_feedback_msg.wrench.torque.y = moment.y();
    m_force_feedback_msg.wrench.torque.z = moment.z();

    m_forcefeedbackPubPtr->publish(m_force_feedback_msg);
}
