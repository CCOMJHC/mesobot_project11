#include "mesobot_project11/mesobot_plugin.h"
#include <pluginlib/class_list_macros.h>

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

  connect(ui_.commandTopicLineEdit, &QLineEdit::editingFinished, this, &MesobotPlugin::on_commandTopicLineEdit_editingFinished);
  
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
}

void MesobotPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString mesobotCommandTopic = instance_settings.value("mesobotCommandTopic", "").toString();
    ui_.commandTopicLineEdit->setText(mesobotCommandTopic);
    //m_ui->helmManager->updateRobotNamespace(robotNamespace);
}

void MesobotPlugin::on_commandTopicLineEdit_editingFinished()
{
  //m_ui->helmManager->updateRobotNamespace(m_ui->robotNamespaceLineEdit->text());
}

} // namespace mesobot

PLUGINLIB_EXPORT_CLASS(mesobot::MesobotPlugin, rqt_gui_cpp::Plugin)
