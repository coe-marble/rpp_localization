/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016  Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_COMMON_HPP_
#define RPP_LOCALIZATION__FILTER_COMMON_HPP_

namespace rpp_localization
{

/**
 * @brief Enumeration that defines the state vector
 */
enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

/**
 * @brief Enumeration that defines the control vector
 */
enum ControlMembers
{
  ControlMemberVx,
  ControlMemberVy,
  ControlMemberVz,
  ControlMemberVroll,
  ControlMemberVpitch,
  ControlMemberVyaw
};

/**
 * @brief Global constants that define our state
 * vector size and offsets to groups of values
 * within that state.
 */
const int STATE_SIZE = 15;
const int POSITION_OFFSET = StateMemberX;
const int ORIENTATION_OFFSET = StateMemberRoll;
const int POSITION_V_OFFSET = StateMemberVx;
const int ORIENTATION_V_OFFSET = StateMemberVroll;
const int POSITION_A_OFFSET = StateMemberAx;

/**
 * @brief Pose and twist messages each contain six variables
 */
const int POSE_SIZE = 6;
const int TWIST_SIZE = 6;
const int POSITION_SIZE = 3;
const int ORIENTATION_SIZE = 3;
const int LINEAR_VELOCITY_SIZE = 3;
const int ACCELERATION_SIZE = 3;
const int ANGULAR_VELOCITY_SIZE = 3;

/**
 * @brief Common constants
 */
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_COMMON_HPP_
