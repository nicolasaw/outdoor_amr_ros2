// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include "mapviz/mapviz_application.h"

#include "rclcpp/rclcpp.hpp"

namespace mapviz
{
  MapvizApplication::MapvizApplication(int& argc, char** argv, rclcpp::Logger logger) :
    QApplication(argc, argv),
    logger_(logger)
  {
  }

  bool MapvizApplication::notify(QObject* receiver, QEvent* event)
  {
    try {
      return QApplication::notify(receiver, event);
    }
    catch (const rclcpp::exceptions::RCLError& e) {
      RCLCPP_ERROR(logger_,
        "Unhandled RCLError in Qt event loop: %s", e.what());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(logger_,
        "Unhandled std::exception in Qt event loop: %s", e.what());
    }

    return false;
  }

  void MapvizApplication::setLogger(const rclcpp::Logger &logger)
  {
    logger_ = logger;
  }
}   // namespace mapviz
