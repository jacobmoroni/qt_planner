#include "mapwidget.hpp"
#include "helper.hpp"
#include "obstaclemap.hpp"
#include "state.hpp"

#include <QPainter>
#include <QTimer>
#include <QKeyEvent>
#include <QWheelEvent>

MapWidget::MapWidget(QWidget *parent, Helper *helper, State *state):
  QWidget(parent), helper(helper), m_state{state}
{
}

MapWidget::~MapWidget()
{
}

void MapWidget::animate()
{
  update();
}

void MapWidget::paintEvent(QPaintEvent *event)
{
  QPainter painter;
  painter.begin(this);
  painter.setRenderHint(QPainter::Antialiasing);
  helper->paint(&painter, event);
  painter.end();
}

void MapWidget::resizeEvent(QResizeEvent *)
{
  helper->updateWidgetSize(this->size());
}

void MapWidget::mouseMoveEvent(QMouseEvent *event)
{
  if (m_move_now == true)
  {
    auto pixel_ratio{this->devicePixelRatio()};
    int x_move{event->x()-m_previous_x};
    int y_move{event->y()-m_previous_y};
    helper->updateTranslation(x_move*pixel_ratio,y_move*pixel_ratio);
    m_previous_x = event->x();
    m_previous_y = event->y();
  }
}

void MapWidget::mousePressEvent(QMouseEvent *event)
{
  unsigned int button{this->getMouseButtonNumber(event)};
  if (button == 3 || button == 2)
  {
    m_previous_x = event->x();
    m_previous_y = event->y();
    m_move_now = true;
  }
  else
    m_move_now = false;

}

void MapWidget::mouseReleaseEvent(QMouseEvent* event)
{
  unsigned int button{this->getMouseButtonNumber(event)};
  if (button==1)
  {
    helper->setGoalPointFromPixel(std::vector<double>{static_cast<double>(event->x()), static_cast<double>(event->y())});
    emit goalUpdated();
  }
}

void MapWidget::wheelEvent(QWheelEvent* event)
{
  event->accept();
  int delta{event->delta()};
  helper->updateScale(delta);
}

unsigned int MapWidget::getMouseButtonNumber(QMouseEvent* event)
{
  unsigned int left_mouse_button{1};
  unsigned int middle_mouse_button{2};
  unsigned int right_mouse_button{3};
  unsigned int button{0};

  switch(event->button())
  {
  case Qt::LeftButton:
    button = left_mouse_button;
    break;

  case Qt::MiddleButton:
    button = middle_mouse_button;
    break;

  case Qt::RightButton:
    button = right_mouse_button;
    break;

  default:
    break;
  }

  return button;
}

void MapWidget::setUnknownAsObstacles(bool new_value)
{
  m_state->obstacles->setUnknownAsObstacles(new_value);
}
