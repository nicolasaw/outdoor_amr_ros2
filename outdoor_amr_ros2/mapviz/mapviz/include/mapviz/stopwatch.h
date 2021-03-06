// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#ifndef MAPVIZ__STOPWATCH_H_
#define MAPVIZ__STOPWATCH_H_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>


namespace mapviz
{
/* This class measures the wall time of an interval and keeps track of
 * the number of intervals, the average duration, and the maximum
 * duration.  This is used to provide some simple measurements to keep
 * an eye on performance.
 */
class Stopwatch
{
 public:
  Stopwatch()
    :
    count_(0),
    clock(),
    total_time_(0, 0),
    max_time_(0, 0),
    start_(0, 0)
  {
  }

  /* Start measuring a new time interval. */
  void start()
  {
    start_ = clock.now();
  }

  /* End the current time interval and update the measurements.
   * Behavior is undefined if start() was not called prior to this.
   */
  void stop()
  {
    rclcpp::Duration dt = clock.now() - start_;
    count_ += 1;
    total_time_ = total_time_ + dt;
    max_time_ = std::max(max_time_, dt);
  }

  /* Return the number of intervals measured. */
  int count() const { return count_; }

  /* Returns the longest observed duration. */
  rclcpp::Duration maxTime() const {return max_time_;}

  /* Returns the average duration spent in the interval. */
  rclcpp::Duration avgTime() const
  {
    if (count_) {
      return total_time_*(1.0/count_);
    } else {
      return rclcpp::Duration(0, 0);
    }
  }

  /* Print measurement info to the ROS console. */
  void printInfo(rclcpp::Logger logger, const std::string &name) const
  {
    if (count_) {
      RCLCPP_INFO(logger,
                "%s -- calls: %d, avg time: %.2fms, max time: %.2fms",
                name.c_str(),
                count_,
                avgTime().seconds()*1000.0,
                maxTime().seconds()*1000.0);
    } else {
      RCLCPP_INFO(logger,
                "%s -- calls: %d, avg time: --ms, max time: --ms",
                name.c_str(),
                count_);
    }
  }

 private:
  int count_;
  rclcpp::Clock clock;
  rclcpp::Duration total_time_;
  rclcpp::Duration max_time_;

  rclcpp::Time start_;
};  // class PluginInstrumentation
}  // namespace mapviz

#endif  // MAPVIZ__STOPWATCH_H_
