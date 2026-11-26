/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

namespace rpp_localization
{
// Coordinate ordering belongs to the navigation model, not the generic filters.
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

// The first six prepared model inputs are linear/angular accelerations.
enum ControlMembers
{
  ControlMemberAx = 0,
  ControlMemberAy,
  ControlMemberAz,
  ControlMemberAroll,
  ControlMemberApitch,
  ControlMemberAyaw
};

inline constexpr int STATE_SIZE = 15;
}  // namespace rpp_localization
