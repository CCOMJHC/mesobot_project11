#ifndef MESOBOT_PROJECT11_MESOBOT_PLUGIN_H
#define MESOBOT_PROJECT11_MESOBOT_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include "ui_mesobot_plugin.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

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


private slots:
  void on_commandTopicLineEdit_editingFinished();
  void on_feedbackTopicLineEdit_editingFinished();
  void on_sendCommandPushButton_pressed();
  void on_queueDrivePushButton_pressed();
  void on_queueGoalPushButton_pressed();
  void on_goalSetPointLineEdit_editingFinished();
  void on_goalDegreesSetPointLineEdit_editingFinished();

private:
  void feedbackCallback(const std_msgs::String::ConstPtr & message);
  void appendFeedback();

  Ui::MesobotPlugin ui_;
  QWidget* widget_=nullptr;


  std::string command_topic_;
  std::string feedback_topic_;
  ros::Publisher raw_publisher_;
  ros::Subscriber raw_subscriber_;

  std::vector<std::string> feedback_to_add_;
  std::mutex feedback_to_add_lock_;

};

} // namespace mesobot

#endif
