#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>

// FAST_LIO in this workspace defaults to the MID-360 + livox_ros_driver2 chain.
// The message contents are compatible, so we keep the existing FAST_LIO call sites
// by aliasing the old namespace to the driver2 messages.
namespace livox_ros_driver = livox_ros_driver2;

namespace livox_msg_compat {
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using CustomMsgConstPtr = livox_ros_driver2::CustomMsg::ConstPtr;
}  // namespace livox_msg_compat