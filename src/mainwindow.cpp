#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "mapwidget.hpp"
#include "subscribeselection.hpp"
#include "obstaclesettings.hpp"
#include "goalpointsettings.hpp"
#include "rrtplanner.hpp"
#include "transforms2d.hpp"
#include "rrtsettings.hpp"
#include "waypointmanager.hpp"
#include "waypointmanagersettings.hpp"
#include "settings.hpp"
#include "state.hpp"
#include "qnode.hpp"
#include "helper.hpp"
#include <QGridLayout>
#include <QTimer>
#include <QToolBar>
#include <QMessageBox>
#include <QListView>
#include <QDockWidget>
#include <QFileDialog>
#include <QListWidget>

namespace qt_planner {

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
  QMainWindow{parent},
  ui{new Ui::MainWindow}
{
  settings = new Settings();
  state = new State();
  qnode = new QNode{argc, argv, settings, state};
  helper = new Helper{settings, state};
  ui->setupUi(this);
  setWindowTitle(tr("Obstacle Map"));
  setupManualUi();
  qnode->initROSCommunication();
  setupTimer();
  setupSignalsAndSlots();
}

MainWindow::~MainWindow()
{
  delete ui;
  delete mapwidget;
  delete goal_settings;
  delete rrt_planner;
  delete waypoint_manager;
  delete helper;
  delete qnode;
}

void MainWindow::setupManualUi()
{
  mapwidget = new MapWidget(this, helper, state);
  waypoint_manager = new WaypointManager(settings, state);
  goal_settings = new GoalPointSettings(this, state);
  rrt_planner = new RRTPlanner(settings, state);
  createZoomToolbar();
  createNavigationToolbar();
  createRosToolbar();
  createPlanningToolbar();
  createLogDockWidget();
  createGoalDockWidget();
  setCentralWidget(mapwidget);
}

void MainWindow::setupTimer()
{
  QTimer *timer{new QTimer(this)};
  connect(timer, &QTimer::timeout,
          mapwidget, &MapWidget::animate);
  connect(timer, &QTimer::timeout,
          goal_settings, &GoalPointSettings::showCurrentPosition);
  timer->start(50);
}

void MainWindow::setupSignalsAndSlots()
{
  connect(qnode, &QNode::loggingUpdated,
          this, &MainWindow::updateLoggingView);
  connect(mapwidget, &MapWidget::goalUpdated,
          goal_settings, &GoalPointSettings::updateGoal);
  connect(goal_settings, &GoalPointSettings::deleteWaypoints,
          this, &MainWindow::removeWaypoints);
  connect(goal_settings, &GoalPointSettings::goalPointSent,
          this, &MainWindow::planPath);
  connect(waypoint_manager, &WaypointManager::reachedWaypoint,
          this, &MainWindow::moveToNextWaypoint);
  connect(waypoint_manager, &WaypointManager::goalReached,
          this, &MainWindow::setGoalReached);
  connect(qnode, &QNode::requestCurrentWaypoint,
          this, &MainWindow::getCurrentWaypoint);
  connect(qnode, &QNode::checkPath,
          this, &MainWindow::replanIfCollisionFound);
  qRegisterMetaType<QVector<int>>("QVector<int>");
}

void MainWindow::createZoomToolbar()
{
  QToolBar *zoom_toolbar = addToolBar(tr("Zoom"));
  QAction *expand_action = createExpandAction();
  zoom_toolbar->addAction(expand_action);
  QAction *zoom_in_action = createZoomInAction();
  zoom_toolbar->addAction(zoom_in_action);
  QAction *zoom_out_action = createZoomOutAction();
  zoom_toolbar->addAction(zoom_out_action);
  ui->viewMenu->addAction(zoom_toolbar->toggleViewAction());
}

void MainWindow::createNavigationToolbar()
{
  QToolBar *nav_toolbar = addToolBar(tr("Navigation"));
  QAction *move_up_action = createMoveUpAction();
  nav_toolbar->addAction(move_up_action);
  QAction *move_down_action = createMoveDownAction();
  nav_toolbar->addAction(move_down_action);
  QAction *move_left_action = createMoveLeftAction();
  nav_toolbar->addAction(move_left_action);
  QAction *move_right_action = createMoveRightAction();
  nav_toolbar->addAction(move_right_action);
  ui->viewMenu->addAction(nav_toolbar->toggleViewAction());
}

void MainWindow::createRosToolbar()
{
  ros_toolbar = addToolBar(tr("ROS"));
  QAction *log_action = createLogAction();
  ros_toolbar->addAction(log_action);
  QAction *ros_action = createROSAction();
  ros_toolbar->addAction(ros_action);
  ui->viewMenu->addAction(ros_toolbar->toggleViewAction());
}

void MainWindow::createPlanningToolbar()
{
  QToolBar *ros_toolbar = addToolBar(tr("Planning"));
  QAction *obstacle_action = createObstacleAction();
  ros_toolbar->addAction(obstacle_action);
  QAction *rrt_action = createRRTAction();
  ros_toolbar->addAction(rrt_action);
  QAction *waypoint_settings_action = createWaypointSettingsAction();
  ros_toolbar->addAction(waypoint_settings_action);
  ui->viewMenu->addAction(ros_toolbar->toggleViewAction());
}

void MainWindow::createLogDockWidget()
{
  log_widget = new QDockWidget(tr("ROS Logging"), this);
  log_widget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
  ros_logging = new QListView(log_widget);
  ros_logging->setModel(qnode->loggingModel());
  log_widget->setWidget(ros_logging);
  addDockWidget(Qt::BottomDockWidgetArea, log_widget);
  log_widget->setFloating(true);
  if (log_widget->isFloating())
  {
    log_widget->setMinimumSize(800,100);
  }
  else
  {
    log_widget->setMinimumSize(200,100);
  }
  ui->viewMenu->addAction(log_widget->toggleViewAction());
  log_widget->setVisible(true);
}

void MainWindow::createGoalDockWidget()
{
  QDockWidget *goal_widget = new QDockWidget(tr("Goal Setting"), this);
  goal_widget->setAllowedAreas(Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
  goal_widget->setWidget(goal_settings);
  addDockWidget(Qt::BottomDockWidgetArea, goal_widget);
  ui->viewMenu->addAction(goal_widget->toggleViewAction());
  goal_widget->setVisible(true);
}

QAction* MainWindow::createExpandAction()
{
  const QIcon expand_icon = QIcon(":images/expand.png");
  QAction *expand_action = new QAction(expand_icon, tr("&Zoom to Fit (F)"), this);
  expand_action->setShortcut(QKeySequence{tr("F")});
  expand_action->setStatusTip(tr("Zoom to Fit (F)"));
  connect(expand_action, &QAction::triggered, this, &MainWindow::zoomToFit);

  return expand_action;
}

void MainWindow::zoomToFit()
{
  helper->zoomToFit();
}

QAction* MainWindow::createZoomInAction()
{
  const QIcon zoom_in_icon = QIcon(":images/zoom_in.png");
  QAction *zoom_in_action = new QAction(zoom_in_icon, tr("&Zoom In (+)"), this);
  zoom_in_action->setShortcut(QKeySequence{tr("+")});
  zoom_in_action->setStatusTip(tr("Zoom In (+)"));
  connect(zoom_in_action, &QAction::triggered, this, &MainWindow::zoomIn);

  return zoom_in_action;
}

void MainWindow::zoomIn()
{
  helper->zoomIn();
}

QAction* MainWindow::createZoomOutAction()
{
  const QIcon zoom_out_icon = QIcon(":images/zoom_out.png");
  QAction *zoom_out_action = new QAction(zoom_out_icon, tr("&Zoom Out (-)"), this);
  zoom_out_action->setShortcut(QKeySequence{tr("-")});
  zoom_out_action->setStatusTip(tr("Zoom Out (-)"));
  connect(zoom_out_action, &QAction::triggered, this, &MainWindow::zoomOut);

  return zoom_out_action;
}

void MainWindow::zoomOut()
{
  helper->zoomOut();
}

QAction* MainWindow::createMoveUpAction()
{
  const QIcon move_up_icon = QIcon(":images/up.png");
  QAction *move_up_action = new QAction(move_up_icon, tr("&Move Up (Up)"), this);
  move_up_action->setShortcut(QKeySequence{tr("Up")});
  move_up_action->setStatusTip(tr("Move Up (Up)"));
  connect(move_up_action, &QAction::triggered, this, &MainWindow::moveUp);

  return move_up_action;
}

void MainWindow::moveUp()
{
  helper->moveUp();
}

QAction* MainWindow::createMoveDownAction()
{
  const QIcon move_down_icon = QIcon(":images/down.png");
  QAction *move_down_action = new QAction(move_down_icon, tr("&Move Down (Down)"), this);
  move_down_action->setShortcut(QKeySequence{tr("Down")});
  move_down_action->setStatusTip(tr("Move Down (Down)"));
  connect(move_down_action, &QAction::triggered, this, &MainWindow::moveDown);

  return move_down_action;
}

void MainWindow::moveDown()
{
  helper->moveDown();
}

QAction* MainWindow::createMoveLeftAction()
{
  const QIcon move_left_icon = QIcon(":images/left.png");
  QAction *move_left_action = new QAction(move_left_icon, tr("&Move Left (Left)"), this);
  move_left_action->setShortcut(QKeySequence{tr("Left")});
  move_left_action->setStatusTip(tr("Move Left (Left)"));
  connect(move_left_action, &QAction::triggered, this, &MainWindow::moveLeft);

  return move_left_action;
}

void MainWindow::moveLeft()
{
  helper->moveLeft();
}

QAction* MainWindow::createMoveRightAction()
{
  const QIcon move_right_icon = QIcon(":images/right.png");
  QAction *move_right_action = new QAction(move_right_icon, tr("&Move Right (Right)"), this);
  move_right_action->setShortcut(QKeySequence{tr("Right")});
  move_right_action->setStatusTip(tr("Move Right (Right)"));
  connect(move_right_action, &QAction::triggered, this, &MainWindow::moveRight);

  return move_right_action;
}

void MainWindow::moveRight()
{
  helper->moveRight();
}

QAction* MainWindow::createLogAction()
{
  const QIcon log_icon = QIcon(":images/logger.png");
  QAction *log_action = new QAction(log_icon, tr("&Hide Logger (L)"), this);
  log_action->setShortcut(QKeySequence{tr("L")});
  log_action->setStatusTip(tr("Hide Logger (L)"));
  log_action->setCheckable(true);
  log_action->setChecked(true);
  connect(log_action, &QAction::triggered, this, &MainWindow::toggleLoggerView);
  return log_action;
}

void MainWindow::toggleLoggerView()
{
  log_widget->setVisible(!log_widget->isVisible());
  ros_toolbar->actions().first()->setChecked(log_widget->isVisible());
  if (log_widget->isVisible())
  {
    ros_toolbar->actions().first()->setToolTip("Hide Logger (L)");
    ros_toolbar->actions().first()->setStatusTip(tr("Hide Logger (L)"));
  }
  else
  {
    ros_toolbar->actions().first()->setToolTip("Show Logger (L)");
    ros_toolbar->actions().first()->setStatusTip(tr("Show Logger (L)"));
  }
}

QAction* MainWindow::createROSAction()
{
  const QIcon ros_icon = QIcon(":images/ros.png");
  QAction *ros_action = new QAction(ros_icon, tr("&Manage ROS Subscriptions (R)"), this);
  ros_action->setShortcut(QKeySequence{tr("R")});
  ros_action->setStatusTip(tr("Manage ROS Subscriptions (R)"));
  connect(ros_action, &QAction::triggered, this, &MainWindow::updateROSSubscription);

  return ros_action;
}

void MainWindow::updateROSSubscription()
{
  std::string gridmap_topics{qnode->getOccupancyGridTopics()};
  std::string waypoint_topics{qnode->getMultiArrayTopics()};
  SubscribeSelection* ros_sub = new SubscribeSelection(this, gridmap_topics, waypoint_topics, settings);
  if (ros_sub->exec()==QDialog::Accepted)
  {
     qnode->initROSCommunication();
  }
}

QAction* MainWindow::createObstacleAction()
{
  const QIcon obstacle_icon = QIcon(":images/obstacle.png");
  QAction *obstacle_action = new QAction(obstacle_icon, tr("&Obstacle Settings (O)"), this);
  obstacle_action->setShortcut(QKeySequence{tr("O")});
  obstacle_action->setStatusTip(tr("Obstacle Settings (O)"));
  connect(obstacle_action, &QAction::triggered, this, &MainWindow::updateObstacleSettings);

  return obstacle_action;
}

void MainWindow::updateObstacleSettings()
{
  ObstacleSettings* obstacle_settings = new ObstacleSettings(this, settings);
  if (obstacle_settings->exec()==QDialog::Accepted)
  {
    mapwidget->setUnknownAsObstacles(settings->obstacle->unknown_as_obstacles);
  }
}

QAction* MainWindow::createRRTAction()
{
  const QIcon rrt_icon = QIcon(":images/rrt.png");
  QAction *rrt_action = new QAction(rrt_icon, tr("&RRT Planner Settings (P)"), this);
  rrt_action->setShortcut(QKeySequence{tr("P")});
  rrt_action->setStatusTip(tr("RRT Planning Settings (P)"));
  connect(rrt_action, &QAction::triggered, this, &MainWindow::updateRRTSettings);

  return rrt_action;
}

void MainWindow::updateRRTSettings()
{
  RRTSettings* rrt_settings = new RRTSettings(this, settings);
  if (rrt_settings->exec()==QDialog::Accepted){}
}

QAction* MainWindow::createWaypointSettingsAction()
{
  const QIcon waypoint_icon = QIcon(":images/waypoint.png");
  QAction *waypoint_action = new QAction(waypoint_icon, tr("&Waypoint Manager Settings (P)"), this);
  waypoint_action->setShortcut(QKeySequence{tr("P")});
  waypoint_action->setStatusTip(tr("Waypoint Manager Settings (P)"));
  connect(waypoint_action, &QAction::triggered, this, &MainWindow::updateWaypointSettings);

  return waypoint_action;
}

void MainWindow::updateWaypointSettings()
{
  WaypointManagerSettings* waypoint_settings = new WaypointManagerSettings(this, settings);
  if (waypoint_settings->exec()==QDialog::Accepted){}
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
}

void MainWindow::on_actionHelp_triggered()
{
  QMessageBox::about(this, tr("Help"),tr("<h2>RRT Path Planner</h2>"
                                         "<p>Make sure a ROS Master is running, a nav_msgs/GridMap topic is being published,"
                                         " and the robot's location is being published through tf for planner to work correctly. </p>"
                                         "<p>Select grid map topic and tf frames from 'Manage ROS Subscriptions' menu.</p>"
                                         "<p>Left click to select goal. </p>"
                                         "<p>Right click or center click and drag to pan map. </p>"
                                         "<p>Scroll to zoom.</p>"
                                         "<p>When goal is selected, click 'Send Goal' to plan path and fly to goal. </p>"
                                         "<p>Click 'Cancel' at any time to stop flight.</p>"));
}

void MainWindow::on_actionSave_Parameters_triggered()
{
  if (!m_param_file.contains(".yaml"))
  {
    m_param_file = QFileDialog::getSaveFileName(this, tr("Save File"),m_param_file,tr("Param Files (*.yaml)"));
  }
  qnode->saveParametersToFile(m_param_file.toUtf8().constData());
}

void qt_planner::MainWindow::on_actionSave_Parameters_As_triggered()
{
  m_param_file = QFileDialog::getSaveFileName(this, tr("Save File"),m_param_file,tr("Param Files (*.yaml)"));
  qnode->saveParametersToFile(m_param_file.toUtf8().constData());
}

void MainWindow::on_actionLoad_Parameters_triggered()
{
  m_param_file = QFileDialog::getOpenFileName(this, tr("Open File"),m_param_file,tr("Param Files (*.yaml)"));
  qnode->setParametersFromFile(m_param_file.toUtf8().constData());
}

void MainWindow::updateLoggingView()
{
  ros_logging->scrollToBottom();
}

void MainWindow::planPath()
{
  rrt_planner->initializeTree();
  bool valid_start{rrt_planner->checkIfCurrentLocationIsValid()};
  bool valid_goal{rrt_planner->checkIfGoalIsValid()};
  if (valid_start)
  {
    if (valid_goal)
    {
      bool reached_goal{rrt_planner->expandTreeToGoal()};
      if (reached_goal)
      {
        std::vector<std::vector<double>> smoothed_path{rrt_planner->getSmoothedPath()};
        helper->setPath(smoothed_path);
        waypoint_manager->generateWaypointsFromPath(smoothed_path);
        qnode->log(QNode::Info, "Found path to goal, flying now.");
        m_at_goal_point = false;
      }
      else
      {
        qnode->log(QNode::Error, "Failed to reach goal, RRT timed out.");
        clearGoal();
      }
    }
    else
    {
      qnode->log(QNode::Error, "Goal point is too close to an obstacle, try a different one.");
      clearGoal();
    }
  }
  else
  {
    qnode->log(QNode::Error, "Bad Starting Point, it is too close to an obstacle.");
    clearGoal();
  }
}

void MainWindow::clearGoal()
{
  goal_settings->clearGoal();
  waypoint_manager->setDefaultWaypoint(Waypoint{state->position->north,
                                                state->position->east,
                                                state->position->down,
                                                state->position->yaw,
                                                settings->waypoint_manager->position_threshold});
}

void MainWindow::removeWaypoints()
{
  std::vector<std::vector<double>> empty_path{};
  helper->setPath(empty_path);
  waypoint_manager->clearWaypoints();
  clearGoal();
}

void MainWindow::replanIfCollisionFound()
{
  rrt_planner->updateObstaclesNED();
  if (rrt_planner->checkPathForCollisions(waypoint_manager->getWaypoints()))
  {
    qnode->log(QNode::Info, "Found Obstacles In Path, Replanning.");
    waypoint_manager->clearWaypoints();
    planPath();
    getCurrentWaypoint();
  }
}

void MainWindow::moveToNextWaypoint()
{
  qnode->log(QNode::Info, "Reached waypoint, moving to next one.");
  helper->moveToNextWaypoint();
  replanIfCollisionFound();
}

void MainWindow::getCurrentWaypoint()
{
  Waypoint current_waypoint{waypoint_manager->getCurrentWaypoint()};
  state->current_waypoint = &current_waypoint;
  qnode->updateWaypoint(current_waypoint);
  goal_settings->showCurrentWaypoint();
}

void MainWindow::setGoalReached()
{
  goal_settings->clearGoal();
  if (!m_at_goal_point)
  {
    qnode->log(QNode::Info, "Goal reached.");
    m_at_goal_point = true;
  }
}
}
