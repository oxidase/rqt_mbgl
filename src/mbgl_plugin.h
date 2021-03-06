#ifndef RQT_MBGL_MAPBOX_GL_PLUGIN_H
#define RQT_MBGL_MAPBOX_GL_PLUGIN_H

#include "mbgl_map_window.h"

#include <rqt_gui_cpp/plugin.h>

#include <ros/node_handle.h>
#include <gps_common/GPSFix.h>

namespace rqt_mbgl
{

class MapboxGLPlugin : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    MapboxGLPlugin();

    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;
    void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                      qt_gui_cpp::Settings& instance_settings) const override ;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                         const qt_gui_cpp::Settings& instance_settings) override;

    bool hasConfiguration() const override;
    void triggerConfiguration() override;

private:
    // Here is a naked pointer with a potential memory leak to prevent Python error
    // RuntimeError: wrapped C/C++ object of type WindowChangedSignaler has been deleted
    rqt_mbgl::MapboxGLMapWindow *widget;

    ros::NodeHandle nh;

    GPSFixes fixes_queue;
    ros::Subscriber extended_fix_sub;
    void extended_fix_callback(const gps_common::GPSFix& fix);
};

} // namespace rqt_mbgl

#endif
