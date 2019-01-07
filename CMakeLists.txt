cmake_minimum_required(VERSION 3.4)

project(qt_planner)

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(catkin REQUIRED COMPONENTS qt_build roscpp tf rosflight_msgs)

get_target_property(QtCore_location Qt5::Core LOCATION)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Core_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    include/qt_planner
                    include/planning_lib)

catkin_package(INCLUDE_DIRS    include
               DEPENDS         rosflight_msgs
               CATKIN_DEPENDS  rosflight_msgs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE MYSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB MYFORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB MYRESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE MYMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qt_planner/*.hpp)

QT5_WRAP_UI(QT_FORMS_HPP ${MYFORMS})

set(TEST_NAME ${PROJECT_NAME}_UnitTest)

file(GLOB PLANNING_LIB RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/planning_lib/*)

add_library(PlanningLib STATIC
    ${PLANNING_LIB}
    )

target_link_libraries(PlanningLib
                      Qt5::Core)

add_executable(${PROJECT_NAME}
               ${MYSOURCE}
               ${MYRESOURCES}
               ${MYMOC}
               ${QT_FORMS_HPP})

target_link_libraries(${PROJECT_NAME}
                      PlanningLib
                      ${QT_LIBRARIES}
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui
                      Qt5::Core)

install(TARGETS qt_planner RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${TEST_NAME}
    ../qt_planner/tests/map_unittests.cpp
    ../qt_planner/tests/transforms2d_unittests.cpp
    ../qt_planner/tests/rrt_planner_unittests.cpp
    ../qt_planner/tests/waypointmanager_unittests.cpp
    ../qt_planner/tests/test_functions.h
    ../qt_planner/tests/test_functions.cpp)

target_link_libraries(${TEST_NAME}
                      Qt5::Core
                      gtest
                      gtest_main
                      PlanningLib)
