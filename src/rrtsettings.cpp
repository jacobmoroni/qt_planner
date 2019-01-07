#include "rrtsettings.hpp"
#include "settings.hpp"
#include "ui_rrtsettings.h"

RRTSettings::RRTSettings(QWidget *parent, Settings *settings) :
  m_settings{settings},
  QDialog{parent},
  ui{new Ui::RRTSettings}
{
  ui->setupUi(this);
  ui->boundary_buffer_spinbox->setToolTip("Buffer size around obstacles where the planner is allowed to search");
  ui->buffer_label->setToolTip("Buffer size around obstacles where the planner is allowed to search");
  ui->boundary_buffer_spinbox->setValue(m_settings->rrt->boundary_buffer);
  ui->timeout_spinbox->setToolTip("Number of iterations that the planner will attempt \nto find goal before throwing an error");
  ui->timeout_label->setToolTip("Number of iterations that the planner will attempt \nto find goal before throwing an error");
  ui->timeout_spinbox->setValue(m_settings->rrt->timeout);
  ui->expand_distance_spinbox->setToolTip("Length of tree expansion on each iteration");
  ui->expand_distance_label->setToolTip("Length of tree expansion on each iteration");
  ui->expand_distance_spinbox->setValue(m_settings->rrt->expand_distance);
  ui->sample_rate_spinbox->setToolTip("Probability percent chance of sampling \ngoal point rather than random number to expand tree");
  ui->sample_rate_label->setToolTip("Probability percent chance of sampling \ngoal point rather than random number to expand tree");
  ui->sample_rate_spinbox->setValue(m_settings->rrt->goal_sample_rate);
}

RRTSettings::~RRTSettings()
{
  delete ui;
}

void RRTSettings::on_expand_distance_spinbox_valueChanged(double arg)
{
  m_settings->rrt->expand_distance = arg;
}

void RRTSettings::on_sample_rate_spinbox_valueChanged(int arg)
{
  m_settings->rrt->goal_sample_rate = arg;
}

void RRTSettings::on_timeout_spinbox_valueChanged(int arg)
{
  m_settings->rrt->timeout = arg;
}

void RRTSettings::on_boundary_buffer_spinbox_valueChanged(double arg)
{
  m_settings->rrt->boundary_buffer = arg;
}
