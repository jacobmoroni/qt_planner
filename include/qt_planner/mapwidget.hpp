#ifndef MAPWIDGET_HPP
#define MAPWIDGET_HPP

#include <QWidget>

class Helper;
class ObstacleMap;
class State;

class MapWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MapWidget(QWidget *parent = nullptr,
                     Helper *helper = nullptr,
                     State *state = nullptr);
  ~MapWidget() override;
  void setUnknownAsObstacles(bool new_value);

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  unsigned int getMouseButtonNumber(QMouseEvent* event);

signals:
  void goalUpdated();

public slots:
  void animate();

private:
  Helper *helper;
  ObstacleMap *m_current_map;
  State *m_state;
  int m_previous_x{0};
  int m_previous_y{0};
  bool m_move_now{false};
};

#endif // MAPWIDGET_HPP
