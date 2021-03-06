#ifndef SUBSCRIBESELECTION_HPP
#define SUBSCRIBESELECTION_HPP

#include <QDialog>

namespace Ui {
class SubscribeSelection;
}

class Settings;

class SubscribeSelection : public QDialog
{
  Q_OBJECT

public:
  explicit SubscribeSelection(QWidget *parent = nullptr,
                              std::string gridmap_topics = nullptr,
                              std::string waypoint_topics = nullptr,
                              Settings *settings = nullptr);
  ~SubscribeSelection();

protected:
  void addTopicsToComboBox(std::string gridmap_topics, std::string waypoint_topics);

private slots:
  void on_comboBox_activated(const QString &arg);
  void on_buttonBox_accepted();
  void on_buttonBox_rejected();
  void on_tf_from_editingFinished();
  void on_tf_to_editingFinished();
  void on_reference_frame_comboBox_currentIndexChanged(int index);

  void on_waypointComboBox_activated(const QString &arg1);

private:
  Ui::SubscribeSelection *ui;
  Settings *m_settings;
};

#endif // SUBSCRIBESELECTION_HPP
