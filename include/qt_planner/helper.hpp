#ifndef HELPER_HPP
#define HELPER_HPP

#include <QBrush>
#include <QFont>
#include <QPen>
#include <QWidget>

class ObstacleMap;
class Settings;
class State;

class Helper
{
public:
    Helper(Settings *settings, State *state);

public:
    void paint(QPainter *painter, QPaintEvent *event);
    void updateScale(int delta);
    void updateTranslation(int x_move, int y_move);
    void zoomToFit();
    void zoomIn();
    void zoomOut();
    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    void updateWidgetSize(QSize size);
    void setGoalPointFromPixel(std::vector<double> new_goal_point);
    void setPath(const std::vector<std::vector<double>> &path);
    void moveToNextWaypoint();


protected:
    void paintBuffer(std::vector<std::vector<double>> obstacles_pixel, QPainter *painter);
    void paintObstacles(std::vector<std::vector<double>> obstacles_pixel, QPainter *painter);
    void paintRobotLocation(QPainter *painter);
    void paintGoal(QPainter *painter);
    void paintPath(QPainter *painter);
    void paintBoudaryBuffer(QPainter *painter);

private:
    QBrush background;
    QBrush obstacle_brush;
    QBrush buffer_brush;
    QBrush robot_brush;
    QBrush goal_brush;
    QBrush path_brush;
    QBrush boundary_brush;

    QFont text_font;
    QPen obstacle_pen;
    QPen buffer_pen;
    QPen robot_pen;
    QPen goal_pen;
    QPen text_pen;
    QPen path_pen;
    QPen boundary_pen;
    std::vector<std::vector<double>> m_robot_indicator{{0,.15},
                                                    {0.3,.4},
                                                    {0,-.4},
                                                    {-.3,.4}};
    double m_scale{20};
    std::vector<double> m_translation{20,20};
    QSize m_widget_size;
    std::vector<double> x_minmax{0,0};
    std::vector<double> y_minmax{0,0};
    Settings *m_settings;
    State *m_state;

    std::vector<std::vector<double>> m_path{};
    static void paintRobotLocation(std::vector<std::vector<double>> robot_indicator, QPainter *painter);
};

#endif // HELPER_HPP
