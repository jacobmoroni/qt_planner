#ifndef RRTSETTINGS_HPP
#define RRTSETTINGS_HPP

#include <QDialog>

namespace Ui {
class RRTSettings;
}

class Settings;

class RRTSettings : public QDialog
{
  Q_OBJECT

public:
  explicit RRTSettings(QWidget *parent = nullptr, Settings *settings = nullptr);
  ~RRTSettings();

private slots:
  void on_expand_distance_spinbox_valueChanged(double arg);
  void on_sample_rate_spinbox_valueChanged(int arg);
  void on_timeout_spinbox_valueChanged(int arg);
  void on_boundary_buffer_spinbox_valueChanged(double arg);

private:
  Ui::RRTSettings *ui;
  Settings *m_settings;
};

#endif // RRTSETTINGS_HPP
