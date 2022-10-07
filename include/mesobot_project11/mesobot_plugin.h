#ifndef MESOBOT_PROJECT11_MESOBOT_PLUGIN_H
#define MESOBOT_PROJECT11_MESOBOT_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include "ui_mesobot_plugin.h"


namespace mesobot {

class MesobotPlugin: public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
  MesobotPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


private:
  void on_commandTopicLineEdit_editingFinished();

  Ui::MesobotPlugin ui_;
  QWidget* widget_=nullptr;

};

} // namespace mesobot

#endif
