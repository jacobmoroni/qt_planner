#ifndef WAYPOINTMANAGERSETTINGS_HPP
#define WAYPOINTMANAGERSETTINGS_HPP

#include <QDialog>

namespace Ui {
class WaypointManagerSettings;
}

class Settings;

class WaypointManagerSettings : public QDialog
{
  Q_OBJECT

public:
  explicit WaypointManagerSettings(QWidget *parent = nullptr, Settings *settings = nullptr);
  ~WaypointManagerSettings();

private slots:
  void on_thresholdSpinBox_valueChanged(double arg);
  void on_yawThresholdSpinBox_valueChanged(double arg);
  void on_threshold2DCheckbox_toggled(bool checked);
  void on_pathCheckFrequencySpinBox_valueChanged(double arg);  
  void on_presetWPCheckBox_toggled(bool checked);

private:
  Ui::WaypointManagerSettings *ui;
  Settings *m_settings;
};

#endif // WAYPOINTMANAGERSETTINGS_HPP
