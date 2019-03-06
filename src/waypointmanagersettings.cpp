#include "waypointmanagersettings.hpp"
#include "ui_waypointmanagersettings.h"
#include "settings.hpp"
#include "transforms2d.hpp"

WaypointManagerSettings::WaypointManagerSettings(QWidget *parent , Settings *settings) :
  QDialog{parent},
  ui{new Ui::WaypointManagerSettings},
  m_settings{settings}
{
  ui->setupUi(this);
  ui->threshold2DCheckbox->setChecked(m_settings->waypoint_manager->threshold_2d);
  ui->threshold2DCheckbox->setToolTip("If checked, waypoint threshold will be calculated only \nfrom planar position rather than 3 dimensional position.");
  ui->thresholdSpinBox->setValue(m_settings->waypoint_manager->position_threshold);
  ui->thresholdSpinBox->setToolTip("Minimum position error required to advance to next waypoint");
  ui->yawThresholdSpinBox->setValue(m_settings->waypoint_manager->yaw_threshold*180.0/PI);
  ui->yawThresholdSpinBox->setToolTip("Minimum yaw error required to advance to next waypoint");
  ui->pathCheckFrequencySpinBox->setValue(m_settings->waypoint_manager->check_path_frequency);
  ui->pathCheckFrequencySpinBox->setToolTip("Approximate rate (in seconds) to recheck path for collisions");
}

WaypointManagerSettings::~WaypointManagerSettings()
{
  delete ui;
}

void WaypointManagerSettings::on_thresholdSpinBox_valueChanged(double arg)
{
  m_settings->waypoint_manager->position_threshold = arg;
}

void WaypointManagerSettings::on_yawThresholdSpinBox_valueChanged(double arg)
{
  m_settings->waypoint_manager->yaw_threshold = arg*PI/180.0;
}

void WaypointManagerSettings::on_threshold2DCheckbox_toggled(bool checked)
{
  m_settings->waypoint_manager->threshold_2d = checked;
}

void WaypointManagerSettings::on_pathCheckFrequencySpinBox_valueChanged(double arg)
{
  m_settings->waypoint_manager->check_path_frequency = arg;
}
