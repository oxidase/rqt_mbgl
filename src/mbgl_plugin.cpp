#include <mbgl_plugin.h>
#include <mbgl_map_window.h>

#include <pluginlib/class_list_macros.h>

#include <QStringList>

namespace rqt_mbgl
{

MapboxGLPlugin::MapboxGLPlugin()
    : rqt_gui_cpp::Plugin()
    , widget(nullptr)
    , nh("~")
{
    std::cout << "MapboxGLPlugin" << "\n";
    setObjectName("MapboxGLPlugin");

    qRegisterMetaType<QMapbox::Coordinates>();
}

void MapboxGLPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    ROS_INFO("MapboxGLPlugin::initPlugin");

    // Access standalone command line arguments
    QStringList argv = context.argv();
    ROS_INFO("argv has %d items", argv.size());
    for (auto s : argv)
    {
        ROS_INFO("   %s", s.toLatin1().data());
    }

    if (!getenv("MAPBOX_ACCESS_TOKEN"))
    {
        ROS_ERROR("MAPBOX_ACCESS_TOKEN is not defined. Please set a token from https://www.mapbox.com/studio/account/tokens/");
    }

    QMapboxGLSettings settings;
    settings.setCacheDatabasePath("/tmp/mbgl-cache.db");
    settings.setCacheDatabaseMaximumSize(20 * 1024 * 1024);

    // Create map widget
    widget = new MapboxGLMapWindow(settings);
    context.addWidget(widget);
    widget->setFocusPolicy(Qt::StrongFocus);
    widget->setFocus(Qt::OtherFocusReason);

    // Subscribe
    extended_fix_sub = nh.subscribe("/extended_fix", 1, &MapboxGLPlugin::extended_fix_callback, this);
}

void MapboxGLPlugin::shutdownPlugin()
{
    ROS_INFO("MapboxGLPlugin::shutdownPlugin");
    // unregister all publishers here
}

void MapboxGLPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                  qt_gui_cpp::Settings& instance_settings) const
{
    widget->saveSettings(instance_settings);
}

void MapboxGLPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                     const qt_gui_cpp::Settings& instance_settings)
{
    widget->restoreSettings(instance_settings);
}

bool MapboxGLPlugin::hasConfiguration() const
{
    return false;
}

void MapboxGLPlugin::triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}

void MapboxGLPlugin::extended_fix_callback(const gps_common::GPSFix& fix)
{
    fixes_queue.push_back({fix.latitude, fix.longitude});
    if (fixes_queue.size() > 10)
        fixes_queue.pop_front();

    QMetaObject::invokeMethod(widget, "showMapMatching", Qt::QueuedConnection, Q_ARG(QMapbox::Coordinates, fixes_queue));
}

}  // namespace rqt_mbgl
PLUGINLIB_DECLARE_CLASS(rqt_mbgl, MapboxGLPlugin, rqt_mbgl::MapboxGLPlugin, rqt_gui_cpp::Plugin)
