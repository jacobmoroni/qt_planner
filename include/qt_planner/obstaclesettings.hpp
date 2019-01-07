#ifndef OBSTACLESETTINGS_HPP
#define OBSTACLESETTINGS_HPP

#include <QDialog>

namespace Ui {
class ObstacleSettings;
}

class Settings;

class ObstacleSettings : public QDialog
{
  Q_OBJECT

public:
  explicit ObstacleSettings(QWidget *parent = nullptr, Settings *settings = nullptr);
  ~ObstacleSettings();

private slots:
    void on_obstacle_spinbox_valueChanged(double arg);
    void on_buffer_spinbox_valueChanged(double arg);
    void on_unknown_as_obs_toggled(bool checked);

private:
  Ui::ObstacleSettings *ui;
  Settings* m_settings;
};

#endif // OBSTACLESETTINGS_HPP
