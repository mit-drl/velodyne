// Copyright (C) 2019 Nick Stathas
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file
 *
 *  Velodyne 3D LIDAR packet time translation classes
 *
 *    These classes provide different methods for translating
 *    a packet's reception time to ROS time.
 *
 *  Classes:
 *
 *     velodyne::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef VELODYNE_DRIVER_TIME_TRANSLATOR_H
#define VELODYNE_DRIVER_TIME_TRANSLATOR_H

#include <ros/ros.h>

#include <velodyne_driver/time_conversion.hpp>

namespace velodyne_driver
{

/** @brief Velodyne time translation base class */
class TimeTranslator
{
public:
  TimeTranslator() = default;
  virtual ~TimeTranslator() = default;

  virtual ros::Time translate(uint8_t const* data, ros::Time time_begin, ros::Time time_recv) = 0;
};

class GPSTimeTranslator : public TimeTranslator
{
public:
  GPSTimeTranslator() = default;

  ros::Time translate(uint8_t const* data, ros::Time time_begin, ros::Time time_recv) override {
    return rosTimeFromGpsTimestamp(data + 1200);
  }
};

class AverageTimeTranslator : public TimeTranslator
{
public:
  AverageTimeTranslator() = default;

  ros::Time translate(uint8_t const* data, ros::Time time_begin, ros::Time time_recv) override {
    return ros::Time((time_begin.toSec() + time_recv.toSec()) / 2.0);
  }
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_TIME_TRANSLATOR_H
