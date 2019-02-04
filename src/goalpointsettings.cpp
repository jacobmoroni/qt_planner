#include "goalpointsettings.hpp"
#include "settings.hpp"
#include "state.hpp"
#include "waypoint.hpp"
#include "ui_goalpointsettings.h"

GoalPointSettings::GoalPointSettings(QWidget *parent, State *state) :
  QWidget{parent},
  m_state{state},
  ui{new Ui::GoalPointSettings}
{
  ui->setupUi(this);
  showCurrentWaypoint();
  m_state->goal_point->down = m_state->current_waypoint->getDown();
  ui->downSpinBox->setValue(m_state->goal_point->down);
}

GoalPointSettings::~GoalPointSettings()
{
  delete ui;
}

void GoalPointSettings::on_northSpinBox_valueChanged(double arg)
{
  m_state->goal_point->north = arg;
}

void GoalPointSettings::on_eastSpinBox_valueChanged(double arg)
{
  m_state->goal_point->east = arg;
}

void GoalPointSettings::on_downSpinBox_valueChanged(double arg)
{
  m_state->goal_point->down = arg;
}

void GoalPointSettings::on_yawSpinBox_valueChanged(double arg)
{
  m_state->goal_point->yaw = arg;
}

void GoalPointSettings::on_goalButton_toggled(bool checked)
{
    if (checked ==true)
    {
      ui->goalButton->setText("Cancel");
      ui->northSpinBox->setDisabled(true);
      ui->northSpinBox->setEnabled(false);
      ui->eastSpinBox->setEnabled(false);
      ui->downSpinBox->setEnabled(false);
      ui->yawSpinBox->setEnabled(false);
      emit goalPointSent();
    }

    if (checked ==false)
    {
      ui->goalButton->setText("Send Goal");
      ui->northSpinBox->setEnabled(true);
      ui->eastSpinBox->setEnabled(true);
      ui->downSpinBox->setEnabled(true);
      ui->yawSpinBox->setEnabled(true);
      emit deleteWaypoints();
    }
}

void GoalPointSettings::updateGoal()
{
  ui->northSpinBox->setValue(m_state->goal_point->north);
  ui->eastSpinBox->setValue(m_state->goal_point->east);
}

void GoalPointSettings::clearGoal()
{
  if (ui->goalButton->isChecked())
  {
    ui->goalButton->toggle();
  }
}

void GoalPointSettings::showCurrentWaypoint()
{
  std::stringstream current_waypoint_str;
  current_waypoint_str.precision(3);
  current_waypoint_str<<"Current Waypoint: [";
  current_waypoint_str<<m_state->current_waypoint->getNorth();
  current_waypoint_str<<", "<<m_state->current_waypoint->getEast();
  current_waypoint_str<<", "<<m_state->current_waypoint->getDown();
  current_waypoint_str<<", "<<m_state->current_waypoint->getYaw()<<"]";

  ui->current_waypoint_label->setText(QString(current_waypoint_str.str().c_str()));
}

void GoalPointSettings::showCurrentPosition()
{
  std::stringstream current_position_str;
  current_position_str.precision(3);
  current_position_str<<"Current Position Estimate: [";
  current_position_str<<m_state->position->north;
  current_position_str<<", "<<m_state->position->east;
  current_position_str<<", "<<m_state->position->down;
  current_position_str<<", "<<m_state->position->yaw<<"]";

  ui->current_position->setText(QString(current_position_str.str().c_str()));
}
