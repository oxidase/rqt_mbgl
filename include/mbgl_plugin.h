#ifndef RQT_MBGL_MAPBOX_GL_PLUGIN_H
#define RQT_MBGL_MAPBOX_GL_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>

#include "mbgl_map_window.h"

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
    MapboxGLMapWindow *widget_;
};

} // namespace rqt_mbgl

#endif
