#include "subscribeselection.hpp"
#include "settings.hpp"
#include "ui_subscribeselection.h"

void SubscribeSelection::addGridmapTopicsToComboBox(std::string gridmap_topics)
{
  QString str = QString::fromUtf8(gridmap_topics.c_str());
  QStringList list = str.split(",");
  m_settings->ros->grid_map_topic = list.at(0).toUtf8().constData();
  ui->comboBox->addItems(list);
}

SubscribeSelection::SubscribeSelection(QWidget *parent,
                                       std::string gridmap_topics,
                                       Settings *settings) :
  QDialog{parent},
  m_settings{settings},
  ui{new Ui::SubscribeSelection}
{
  ui->setupUi(this);
  addGridmapTopicsToComboBox(gridmap_topics);
  ui->tf_from->setText(QString{QString::fromUtf8(settings->ros->tf_from.c_str())});
  ui->tf_to->setText(QString{QString::fromUtf8(settings->ros->tf_to.c_str())});
}

SubscribeSelection::~SubscribeSelection()
{
  delete ui;
}

void SubscribeSelection::on_comboBox_activated(const QString &arg)
{
  m_settings->ros->grid_map_topic = arg.toUtf8().constData();
}

void SubscribeSelection::on_tf_from_editingFinished()
{
  m_settings->ros->tf_from = ui->tf_from->text().toUtf8().constData();
}

void SubscribeSelection::on_tf_to_editingFinished()
{
  m_settings->ros->tf_to = ui->tf_to->text().toUtf8().constData();
}

void SubscribeSelection::on_reference_frame_comboBox_currentIndexChanged(int index)
{
  m_settings->ros->tf_reference_frame = index;
}

void SubscribeSelection::on_buttonBox_rejected()
{
  QDialog::reject();
}

void SubscribeSelection::on_buttonBox_accepted()
{
  QDialog::accept();
}
