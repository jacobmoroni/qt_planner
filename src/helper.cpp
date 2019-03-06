#include "helper.hpp"
#include "qnode.hpp"
#include "settings.hpp"
#include "state.hpp"
#include "obstaclemap.hpp"
#include "transforms2d.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QWidget>
#include <vector>

Helper::Helper(Settings *settings, State *state):
  m_settings{settings},
  m_state{state}
{
  background = QBrush(Qt::white);
  obstacle_brush = QBrush(Qt::black);
  obstacle_pen = QPen(Qt::black);
  obstacle_pen.setWidth(0);
  buffer_brush = QBrush(QColor(100,100,100));
  buffer_pen = QPen(QColor(100,100,100));
  buffer_pen.setWidth(0);
  robot_brush = QBrush(Qt::green);
  robot_pen = QPen(Qt::black);
  robot_pen.setWidth(0);
  goal_brush = QBrush(Qt::red);
  goal_pen = QPen(Qt::black);
  goal_pen.setWidth(0);
  path_brush = QBrush(Qt::blue);
  path_pen = QPen(Qt::blue);
  path_pen.setWidth(0);
  boundary_brush = QBrush(Qt::NoBrush);
  boundary_pen = QPen(Qt::gray);
  boundary_pen.setWidth(0);
  boundary_pen.setStyle(Qt::DotLine);
  text_pen = QPen(QColor(100,100,100));
  text_font.setPixelSize(50);
}

void Helper::paintObstacles(std::vector<std::vector<double>> obstacles_pixel, QPainter *painter)
{
  painter->setBrush(obstacle_brush);
  painter->setPen(obstacle_pen);
  for (std::vector<double> obstacle: obstacles_pixel)
  {
    painter->drawEllipse(QPointF(obstacle[0],obstacle[1]),m_settings->obstacle->obstacle_size, m_settings->obstacle->obstacle_size);
  }
}

void Helper::paintBuffer(std::vector<std::vector<double>> obstacles_pixel, QPainter *painter)
{
  painter->setBrush(buffer_brush);
  painter->setPen(buffer_pen);
  for (std::vector<double> obstacle: obstacles_pixel)
  {
    painter->drawEllipse(QPointF{obstacle[0],obstacle[1]},m_settings->obstacle->buffer_size, m_settings->obstacle->buffer_size);
  }
}

void Helper::paintRobotLocation(QPainter *painter)
{
  painter->setBrush(robot_brush);
  painter->setPen(robot_pen);
  QPointF indicator[4];
  std::vector<std::vector<double>> robot_indicator{convertVectorNWU2Pixel(m_robot_indicator)};
  std::vector<std::vector<double>> robot_indicator_pixel{{}};
  if (m_settings->ros->tf_reference_frame == m_settings->ros->NED)
    robot_indicator = rotateVector(robot_indicator, m_state->position->yaw*180/PI);
  else if (m_settings->ros->tf_reference_frame == m_settings->ros->NWU)
    robot_indicator = rotateVector(robot_indicator, m_state->position->yaw*-180/PI);
  robot_indicator = translateVector(robot_indicator,std::vector<double>{m_state->position->north, m_state->position->east});
  robot_indicator_pixel = convertVectorNED2Pixel(robot_indicator);

  for (unsigned int i{0}; i<static_cast<unsigned int>(robot_indicator_pixel.size()); i++)
  {
    indicator[i] = QPointF(robot_indicator_pixel[i][0], robot_indicator_pixel[i][1]);
  }

  painter->drawPolygon(indicator,4);
}

void Helper::paintGoal(QPainter *painter)
{
  painter->setBrush(goal_brush);
  painter->setPen(goal_pen);
  QPointF indicator[4];
  std::vector<double> goal_point_pixel{convertPointNED2Pixel(std::vector<double>{m_state->goal_point->north, m_state->goal_point->east})};
  std::vector<std::vector<double>> goal_indicator_ned{scaleVector(m_robot_indicator, 0.8)};
  goal_indicator_ned = rotateVector(goal_indicator_ned, m_state->goal_point->yaw);
  goal_indicator_ned = translateVector(goal_indicator_ned, goal_point_pixel);
  std::vector<std::vector<double>> goal_indicator_pixel{goal_indicator_ned};
  for (unsigned int i{0}; i<static_cast<unsigned int>(goal_indicator_pixel.size()); i++)
  {
    indicator[i] = QPointF(goal_indicator_pixel[i][0], goal_indicator_pixel[i][1]);
  }

  painter->drawPolygon(indicator,4);
}

void Helper::paintPath(QPainter *painter)
{
  painter->setBrush(path_brush);
  painter->setPen(path_pen);
  std::vector<std::vector<double>> path_pixel{convertVectorNED2Pixel(m_path)};
  std::vector<double> robot_location{};
  robot_location = convertPointNED2Pixel(std::vector<double>{m_state->position->north, m_state->position->east});

  if (path_pixel.size() > 1)
  {
    painter->drawEllipse(QPointF{path_pixel[1][0], path_pixel[1][1]}, .1,.1);
    painter->drawLine(QPointF{robot_location[0],robot_location[1]}, QPointF{path_pixel[1][0], path_pixel[1][1]});
  }
  for (unsigned int i{1}; i<static_cast<unsigned int>(path_pixel.size()-1); i++)
  {
    painter->drawEllipse(QPointF{path_pixel[i+1][0], path_pixel[i+1][1]}, .1,.1);
    painter->drawLine(QPointF{path_pixel[i][0], path_pixel[i][1]}, QPointF{path_pixel[i+1][0], path_pixel[i+1][1]});

  }
}

void Helper::paintBoudaryBuffer(QPainter *painter)
{
  painter->setBrush(boundary_brush);
  painter->setPen(boundary_pen);
  QPointF max{x_minmax[1]+m_settings->rrt->boundary_buffer, y_minmax[1]+m_settings->rrt->boundary_buffer};
  QPointF min{x_minmax[0]-m_settings->rrt->boundary_buffer, y_minmax[0]-m_settings->rrt->boundary_buffer};
  painter->drawRect(QRectF{max,min});
}

void Helper::paint(QPainter *painter, QPaintEvent *event)
{
  std::vector<std::vector<double>> obstacles_grid{m_state->obstacles->getObstacleLocations()};
  std::vector<std::vector<double>> obstacles_pixel{convertVectorGrid2Pixel(obstacles_grid)};
  x_minmax = getXMinMax(obstacles_pixel);
  y_minmax = getYMinMax(obstacles_pixel);
  painter->fillRect(event->rect(), background);
  painter->translate(m_translation[0], m_translation[1]);
  painter->scale(m_scale,m_scale);
  painter->save();
  paintBuffer(obstacles_pixel, painter);
  paintObstacles(obstacles_pixel, painter);
  paintRobotLocation(painter);
  paintGoal(painter);
  if (m_path.size() > 0)
    paintPath(painter);
  paintBoudaryBuffer(painter);
  painter->restore();
}

void Helper::updateScale(int delta)
{
  m_scale = m_scale*(1+delta*.001);
}

void Helper::updateTranslation(int x_move, int y_move)
{
  m_translation = {m_translation[0]+x_move, m_translation[1]+y_move};

}

void Helper::updateWidgetSize(QSize size)
{
  m_widget_size = size;
}

void Helper::zoomToFit()
{
  double x_range{fabs(x_minmax[1]-x_minmax[0])};
  double y_range{fabs(y_minmax[1]-y_minmax[0])};
  double y_scale = (m_widget_size.height()/(y_range+1))*0.9;
  double x_scale = (m_widget_size.width()/(x_range+1))*0.9;
  if (x_scale>y_scale)
    m_scale = y_scale;
  else
    m_scale = x_scale;
  double x_translate = m_widget_size.width()/2.0-((x_range*m_scale)/2.0+x_minmax[0]*m_scale);
  double y_translate = m_widget_size.height()/2.0-((y_range*m_scale)/2.0+y_minmax[0]*m_scale);
  m_translation = {x_translate,y_translate};
}

void Helper::zoomIn()
{
  m_scale = m_scale*1.1;
}

void Helper::zoomOut()
{
  m_scale = m_scale*0.9;
}

void Helper::moveUp()
{
  m_translation = {m_translation[0], m_translation[1]+10};
}

void Helper::moveDown()
{
  m_translation = {m_translation[0], m_translation[1]-10};
}

void Helper::moveLeft()
{
  m_translation = {m_translation[0]+10, m_translation[1]};
}

void Helper::moveRight()
{
  m_translation = {m_translation[0]-10, m_translation[1]};
}

void Helper::setGoalPointFromPixel(std::vector<double> new_goal_point)
{
  std::vector<double> goal_point_pixel = {(new_goal_point[0] - m_translation[0]) / m_scale, (new_goal_point[1] - m_translation[1]) / m_scale};
  std::vector<double>goal_point_ned{convertPointPixel2NED(goal_point_pixel)};
  m_state->goal_point->north = goal_point_ned[0];
  m_state->goal_point->east = goal_point_ned[1];
}

void Helper::setPath(const std::vector<std::vector<double>> &path)
{
  m_path = path;
}

void Helper::moveToNextWaypoint()
{
  m_path.erase(m_path.begin());
}
