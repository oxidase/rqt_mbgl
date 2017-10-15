#include <mbgl_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_mbgl
{

MapboxGLPlugin::MapboxGLPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MapboxGLPlugin");
}

void MapboxGLPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
}

void MapboxGLPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MapboxGLPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MapboxGLPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_mbgl
PLUGINLIB_DECLARE_CLASS(rqt_mbgl, MapboxGLPlugin, rqt_mbgl::MapboxGLPlugin, rqt_gui_cpp::Plugin)
