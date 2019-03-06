#ifndef GOALPOINTSETTINGS_HPP
#define GOALPOINTSETTINGS_HPP

#include <QWidget>

namespace Ui {
class GoalPointSettings;
}
class State;
class Waypoint;
class GoalPointSettings : public QWidget
{
  Q_OBJECT

public:
  explicit GoalPointSettings(QWidget *parent = nullptr, State *state = nullptr);
  ~GoalPointSettings();
  void clearGoal();
  void showCurrentWaypoint();

signals:
  void goalPointSent();
  void deleteWaypoints();

public slots:
  void updateGoal();
  void showCurrentPosition();

private slots:
  void on_northSpinBox_valueChanged(double arg);
  void on_eastSpinBox_valueChanged(double arg);
  void on_downSpinBox_valueChanged(double arg);
  void on_yawSpinBox_valueChanged(double arg);
  void on_goalButton_toggled(bool checked);

private:
  Ui::GoalPointSettings *ui;
  State *m_state;
};

#endif // GOALPOINTSETTINGS_HPP
