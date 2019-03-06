#include "obstaclesettings.hpp"
#include "settings.hpp"
#include "ui_obstaclesettings.h"

ObstacleSettings::ObstacleSettings(QWidget *parent, Settings *settings) :
  QDialog{parent},
  ui{new Ui::ObstacleSettings},
  m_settings{settings}
{
  ui->setupUi(this);
  ui->buffer_spinbox->setValue(m_settings->obstacle->buffer_size);
  ui->buffer_spinbox->setToolTip("Buffer radius around obstacles, used for path collision");
  ui->obstacle_spinbox->setValue(m_settings->obstacle->obstacle_size);
  ui->obstacle_spinbox->setToolTip("Obstacle radius to plot in GUI");
  ui->unknown_as_obs->setChecked(m_settings->obstacle->unknown_as_obstacles);
  ui->unknown_as_obs->setToolTip("Set unknown areas of map as obstacles, \nThis will drastically slow simulation down");
}

ObstacleSettings::~ObstacleSettings()
{
  delete ui;
}

void ObstacleSettings::on_obstacle_spinbox_valueChanged(double arg)
{
  m_settings->obstacle->obstacle_size = arg;
}

void ObstacleSettings::on_buffer_spinbox_valueChanged(double arg)
{
  m_settings->obstacle->buffer_size = arg;
}

void ObstacleSettings::on_unknown_as_obs_toggled(bool checked)
{
   m_settings->obstacle->unknown_as_obstacles = checked;
}
