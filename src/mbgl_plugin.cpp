#include <mbgl_plugin.h>
#include <mbgl_map_window.h>

#include <pluginlib/class_list_macros.h>

#include <QStringList>


namespace rqt_mbgl
{

MapboxGLPlugin::MapboxGLPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("MapboxGLPlugin");
}

void MapboxGLPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    ROS_INFO("MapboxGLPlugin::initPlugin");

    // access standalone command line arguments
    QStringList argv = context.argv();
    ROS_INFO("argv has %d items", argv.size());
    for (auto s : argv) {
        ROS_INFO("   %s", s.toLatin1().data());
    }

    QMapboxGLSettings settings;
    settings.setCacheDatabasePath("/tmp/mbgl-cache.db");
    settings.setCacheDatabaseMaximumSize(20 * 1024 * 1024);

    // create map widget
    widget_ = new MapboxGLMapWindow(settings);
    context.addWidget(widget_);
    widget_->setFocusPolicy(Qt::StrongFocus);
    widget_->setFocus(Qt::OtherFocusReason);
}

void MapboxGLPlugin::shutdownPlugin()
{
    ROS_INFO("MapboxGLPlugin::shutdownPlugin");
    // unregister all publishers here
}

void MapboxGLPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                  qt_gui_cpp::Settings& instance_settings) const
{
    auto &map = widget_->getMap();

    instance_settings.setValue("latitude", map.latitude());
    instance_settings.setValue("longitude", map.longitude());
    instance_settings.setValue("zoom", map.zoom());
    instance_settings.setValue("bearing", map.bearing());
    instance_settings.setValue("pitch", map.pitch());
    instance_settings.setValue("styleUrl", map.styleUrl());
    instance_settings.setValue("scale", map.scale());
}

void MapboxGLPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                     const qt_gui_cpp::Settings& instance_settings)
{
    auto &map = widget_->getMap();

    map.setStyleUrl(instance_settings.value("styleUrl", "mapbox://styles/mapbox/streets-v10").toString());
    map.setZoom(instance_settings.value("zoom", 14).toDouble());
    map.setBearing(instance_settings.value("bearing", 0).toDouble());
    map.setPitch(instance_settings.value("pitch", 0).toDouble());
    map.setScale(instance_settings.value("scale", 16384).toDouble());
    map.setLatitude(instance_settings.value("latitude", 48.13727).toDouble());
    map.setLongitude(instance_settings.value("longitude", 11.575506).toDouble());
}

bool MapboxGLPlugin::hasConfiguration() const
{
  return false;
}

void MapboxGLPlugin::triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}

}  // namespace rqt_mbgl
PLUGINLIB_DECLARE_CLASS(rqt_mbgl, MapboxGLPlugin, rqt_mbgl::MapboxGLPlugin, rqt_gui_cpp::Plugin)
