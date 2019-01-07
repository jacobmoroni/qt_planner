#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QListView>
#include <QListWidget>

class Helper;
class MapWidget;
class GoalPointSettings;
class RRTPlanner;
class WaypointManager;
class Settings;
class State;

namespace Ui {
class MainWindow;
}

namespace qt_planner {

class QNode;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
  ~MainWindow();
    
protected:
  void createZoomToolbar();
  QAction* createExpandAction();
  QAction* createZoomInAction();
  QAction* createZoomOutAction();
  void createNavigationToolbar();
  QAction* createMoveUpAction();
  QAction* createMoveDownAction();
  QAction* createMoveLeftAction();
  QAction* createMoveRightAction();
  void createRosToolbar();
  QAction* createLogAction();
  QAction* createROSAction();
  void createPlanningToolbar();
  QAction* createObstacleAction();
  QAction* createRRTAction();
  QAction* createWaypointSettingsAction();
  void createLogDockWidget();
  void createGoalDockWidget();
  void createSaveDockWidget();
  void setupManualUi();
  void setupSignalsAndSlots();
  void setupTimer();
  void clearGoal();

public slots:
  void zoomToFit();
  void zoomIn();
  void zoomOut();
  void moveUp();
  void moveDown();
  void moveLeft();
  void moveRight();
  void toggleLoggerView();
  void updateROSSubscription();
  void updateObstacleSettings();
  void updateRRTSettings();
  void updateWaypointSettings();
  void updateLoggingView();
  void planPath();
  void removeWaypoints();
  void moveToNextWaypoint();
  void getCurrentWaypoint();
  void setGoalReached();
  void replanIfCollisionFound();

private slots:
  void on_actionExit_triggered();
  void on_actionHelp_triggered();
  void on_actionSave_Parameters_triggered();
  void on_actionLoad_Parameters_triggered();

  void on_actionSave_Parameters_As_triggered();

private:
  Ui::MainWindow *ui;
  QToolBar *ros_toolbar;
  QDockWidget *log_widget;
  QListView *ros_logging;
  QListWidget *list_widget;
  MapWidget *mapwidget;
  QNode *qnode;
  Helper *helper;
  GoalPointSettings *goal_settings;
  RRTPlanner *rrt_planner;
  WaypointManager *waypoint_manager;
  Settings *settings;
  State *state;
  QString m_param_file{"/home/jacob/byu_classes/cpp_gui/final-project-jacobmoroni/src/qt_planner/params/"};
  bool m_at_goal_point{true};
};

}
#endif // MAINWINDOW_HPP
