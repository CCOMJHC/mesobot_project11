#include "mesobot_project11/mesobot_plugin.h"
#include <pluginlib/class_list_macros.h>

#include <QDebug>
namespace mesobot
{

MesobotPlugin::MesobotPlugin():rqt_gui_cpp::Plugin()
{
  setObjectName("MesobotPlugin");
}

void MesobotPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  ui_.goalDegreesOfFreedomComboBox->addItem("0 forward");
  ui_.goalDegreesOfFreedomComboBox->addItem("1 starboard (right)");
  ui_.goalDegreesOfFreedomComboBox->addItem("2 depth (down)");
  ui_.goalDegreesOfFreedomComboBox->addItem("3 roll");
  ui_.goalDegreesOfFreedomComboBox->addItem("4 pitch");
  ui_.goalDegreesOfFreedomComboBox->addItem("5 heading (yaw)");

  connect(ui_.commandTopicLineEdit, &QLineEdit::editingFinished, this, &MesobotPlugin::on_commandTopicLineEdit_editingFinished);
  connect(ui_.feedbackTopicLineEdit, &QLineEdit::editingFinished, this, &MesobotPlugin::on_feedbackTopicLineEdit_editingFinished);
  connect(ui_.sendCommandPushButton, &QPushButton::pressed, this, &MesobotPlugin::on_sendCommandPushButton_pressed);
  connect(ui_.queueDrivePushButton, &QPushButton::pressed, this, &MesobotPlugin::on_queueDrivePushButton_pressed);
  connect(ui_.queueGoalPushButton, &QPushButton::pressed, this, &MesobotPlugin::on_queueGoalPushButton_pressed);
  connect(ui_.goalSetPointLineEdit, &QLineEdit::editingFinished, this, &MesobotPlugin::on_goalSetPointLineEdit_editingFinished);
  connect(ui_.goalDegreesSetPointLineEdit, &QLineEdit::editingFinished, this, &MesobotPlugin::on_goalDegreesSetPointLineEdit_editingFinished);
  

  widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  
  context.addWidget(widget_);
}

void MesobotPlugin::shutdownPlugin()
{

}

void MesobotPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString mesobotCommandTopic = ui_.commandTopicLineEdit->text();
  instance_settings.setValue("mesobotCommandTopic", mesobotCommandTopic);
  QString mesobotFeedbackTopic = ui_.feedbackTopicLineEdit->text();
  instance_settings.setValue("mesobotFeedbackTopic", mesobotFeedbackTopic);
  QString smsPreamble = ui_.smsPreambleLineEdit->text();
  instance_settings.setValue("mesobotSMSPreamble", smsPreamble);

}

void MesobotPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString mesobotCommandTopic = instance_settings.value("mesobotCommandTopic", "").toString();
  ui_.commandTopicLineEdit->setText(mesobotCommandTopic);
  QString mesobotFeedbackTopic = instance_settings.value("mesobotFeedbackTopic", "").toString();
  ui_.feedbackTopicLineEdit->setText(mesobotFeedbackTopic);

  QString smsPreamble = instance_settings.value("mesobotSMSPreamble", "").toString();
  ui_.smsPreambleLineEdit->setText(smsPreamble);

  on_commandTopicLineEdit_editingFinished();
  on_feedbackTopicLineEdit_editingFinished();

}

void MesobotPlugin::on_commandTopicLineEdit_editingFinished()
{
  ROS_INFO_STREAM(ui_.commandTopicLineEdit->text().toStdString());
  auto topic = ui_.commandTopicLineEdit->text().toStdString();
  if(topic != command_topic_)
  {
    raw_publisher_.shutdown();
    if(!topic.empty())
      raw_publisher_ = getNodeHandle().advertise<std_msgs::String>(topic, 10);
  }
  command_topic_ = topic;
}

void MesobotPlugin::on_feedbackTopicLineEdit_editingFinished()
{
  ROS_INFO_STREAM(ui_.feedbackTopicLineEdit->text().toStdString());
  auto topic = ui_.feedbackTopicLineEdit->text().toStdString();
  if(topic != feedback_topic_)
  {
    raw_subscriber_.shutdown();
    if(!topic.empty())
      raw_subscriber_ = getNodeHandle().subscribe(topic, 10, &MesobotPlugin::feedbackCallback, this);
    feedback_topic_ = topic;
  }  
}

void MesobotPlugin::feedbackCallback(const std_msgs::String::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(feedback_to_add_lock_);
  feedback_to_add_.push_back(message->data);
  QMetaObject::invokeMethod(this, &MesobotPlugin::appendFeedback);
}

void MesobotPlugin::appendFeedback()
{
  std::lock_guard<std::mutex> lock(feedback_to_add_lock_);
  for(auto feedback: feedback_to_add_)
    ui_.feedbackPlainTextEdit->appendPlainText(feedback.c_str());
  feedback_to_add_.clear();
}

void MesobotPlugin::on_sendCommandPushButton_pressed()
{
  std_msgs::String message;
  message.data = ui_.commandLineEdit->text().toStdString();
  raw_publisher_.publish(message);
  {
    std::lock_guard<std::mutex> lock(feedback_to_add_lock_);
    feedback_to_add_.push_back(message.data);
  }
  QMetaObject::invokeMethod(this, &MesobotPlugin::appendFeedback);
}

void MesobotPlugin::on_queueDrivePushButton_pressed()
{
  QString command = ui_.smsPreambleLineEdit->text();
  command += "DRIVE ";
  command += ui_.driveDutyCycleLineEdit->text();
  command += " ";
  command += ui_.driveTimeLineEdit->text();
  ui_.commandLineEdit->setText(command);
}

void MesobotPlugin::on_queueGoalPushButton_pressed()
{
  QString command = ui_.smsPreambleLineEdit->text();
  command += "GOAL1 ";
  QString dof = ui_.goalDegreesOfFreedomComboBox->currentText();
  if(dof.size() > 1)
    dof = dof.toStdString().substr(0,1).c_str();
  command += dof;
  command += " ";
  command += ui_.goalSetPointLineEdit->text();
  ui_.commandLineEdit->setText(command);

}

void MesobotPlugin::on_goalSetPointLineEdit_editingFinished()
{
  bool ok;
  double radians = ui_.goalSetPointLineEdit->text().toDouble(&ok);
  if(ok)
  {
    auto degrees = 180.0*radians/M_PI;
    ui_.goalDegreesSetPointLineEdit->setText(QString::number(degrees));
  }
}

void MesobotPlugin::on_goalDegreesSetPointLineEdit_editingFinished()
{
  bool ok;
  double degrees = ui_.goalDegreesSetPointLineEdit->text().toDouble(&ok);
  if(ok)
  {
    auto radians = M_PI*degrees/180.0;
    ui_.goalSetPointLineEdit->setText(QString::number(radians));
  }

}


} // namespace mesobot

PLUGINLIB_EXPORT_CLASS(mesobot::MesobotPlugin, rqt_gui_cpp::Plugin)
