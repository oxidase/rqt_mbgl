#include <mbgl_plugin.h>
#include <mbgl_map_window.h>

#include <pluginlib/class_list_macros.h>

#include <QStringList>

namespace
{
const constexpr double DEGREE_TO_RAD = M_PI / 180.;
const constexpr double RAD_TO_DEGREE = 180. / M_PI;
const constexpr double EARTH_RADIUS = 6371008.8; // IUGG  mean radius (2a+b)/3


double computeDistance(const gps_common::GPSFix& from, const gps_common::GPSFix &to)
{
    // Use haversine formula
    const auto lat1 = from.latitude * DEGREE_TO_RAD;
    const auto long1 = from.longitude * DEGREE_TO_RAD;
    const auto lat2 = to.latitude * DEGREE_TO_RAD;
    const auto long2 = to.longitude * DEGREE_TO_RAD;
    const auto dlong = long1 - long2;
    const auto dlat = lat1 - lat2;
    const auto aharv = std::pow(std::sin(dlat / 2.0), 2.0) +
                         std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlong / 2.), 2);
    const auto charv = 2. * std::atan2(std::sqrt(aharv), std::sqrt(1.0 - aharv));
    return EARTH_RADIUS * charv;
}

double computeBearing(const gps_common::GPSFix& from, const gps_common::GPSFix &to)
{
    const auto lon_delta = (to.longitude - from.longitude) * DEGREE_TO_RAD;
    const auto lat1 = from.latitude * DEGREE_TO_RAD;
    const auto lat2 = to.latitude * DEGREE_TO_RAD;
    const auto y = std::sin(lon_delta) * std::cos(lat2);
    const auto x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lon_delta);
    return 180. + std::atan2(y, x) * RAD_TO_DEGREE;
}

}

namespace rqt_mbgl
{

MapboxGLPlugin::MapboxGLPlugin()
    : rqt_gui_cpp::Plugin()
    , widget(nullptr)
    , nh("~")
{
    std::cout << "MapboxGLPlugin" << "\n";
    setObjectName("MapboxGLPlugin");
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
    const auto distance = computeDistance(fix, previous_fix);
    const auto bearing = computeBearing(fix, previous_fix);

    if (distance < 0.5)
        return;

    //std::cout << "map " << map.bearing() << " " << map.latitude() << " " << map.longitude() << "\n";
    //widget->flyTo({40,40},30);
    //widget->flyTo({fix.latitude, fix.longitude}, bearing);

    //map.jumpTo(QMapboxGLCameraOptions{QPointF{fix.latitude, fix.longitude}, QPointF{400, 400}, 15, bearing, 30});
    //map.jumpTo(QMapboxGLCameraOptions{QPointF{fix.latitude, fix.longitude}, QPointF{0, 0}, 15, 0, 30});

    //map.setLatitude(fix.latitude);
    //map.setLongitude(fix.longitude);
    //widget->placeCar({fix.latitude, fix.longitude}, bearing);

    // std::cout << fix.latitude << " " << fix.longitude << " " << bearing << "\n";

    // //map.setScale(instance_settings.value("scale", 16384).toDouble());

    // if (distance > 0.1)
    // {
    //     //map.setBearing(bearing);
    // }

    previous_fix = fix;
}

}  // namespace rqt_mbgl
PLUGINLIB_DECLARE_CLASS(rqt_mbgl, MapboxGLPlugin, rqt_mbgl::MapboxGLPlugin, rqt_gui_cpp::Plugin)
