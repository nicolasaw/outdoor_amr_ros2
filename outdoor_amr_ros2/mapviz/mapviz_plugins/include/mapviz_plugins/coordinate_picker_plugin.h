// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#ifndef MAPVIZ_PLUGINS__COORDINATE_PICKER_PLUGIN_H_
#define MAPVIZ_PLUGINS__COORDINATE_PICKER_PLUGIN_H_

#include <mapviz/mapviz_plugin.h>

// ROS Libraries
#include <rclcpp/rclcpp.hpp>

// Mapviz libraries
#include <mapviz/map_canvas.h>

// C++ Standard Libraries
#include <string>

// QT autogenerated files
#include "ui_coordinate_picker_config.h"

namespace mapviz_plugins
{
class CoordinatePickerPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

  public:
    CoordinatePickerPlugin();
    ~CoordinatePickerPlugin() override;

    bool Initialize(QGLWidget* canvas) override;
    void Shutdown() override { }

    void Draw(double x, double y, double scale) override;
    void Transform() override { }

    void LoadConfig(const YAML::Node& node, const std::string& path) override;
    void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;

    QWidget* GetConfigWidget(QWidget* parent) override;

    void PrintError(const std::string& message) override;
    void PrintInfo(const std::string& message) override;
    void PrintWarning(const std::string& message) override;

  protected:
    bool eventFilter(QObject* object, QEvent* event) override;
    bool handleMousePress(QMouseEvent*);
    bool handleMouseRelease(QMouseEvent*);
    bool handleMouseMove(QMouseEvent*);

  protected Q_SLOTS:
    void SelectFrame();
    void FrameEdited();
    void ToggleCopyOnClick(int state);
    void ClearCoordList();

  private:
    Ui::coordinate_picker_config ui_;
    QWidget* config_widget_;
    mapviz::MapCanvas* map_canvas_;

    bool copy_on_click_;
};
}   // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__COORDINATE_PICKER_PLUGIN_H_

