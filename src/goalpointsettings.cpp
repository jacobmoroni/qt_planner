#include "goalpointsettings.hpp"
#include "settings.hpp"
#include "state.hpp"
#include "ui_goalpointsettings.h"

GoalPointSettings::GoalPointSettings(QWidget *parent, State *state) :
  QWidget{parent},
  m_state{state},
  ui{new Ui::GoalPointSettings}
{
  ui->setupUi(this);
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
