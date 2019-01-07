#include "rrtnode.hpp"

RRTNode::RRTNode(){}
RRTNode::RRTNode(double x, double y, int parent) : m_x{x} , m_y{y} , m_parent_index{parent}
{
}

double RRTNode::getX()
{
  return m_x;
}

void RRTNode::setX(double x)
{
  m_x = x;
}

double RRTNode::getY()
{
  return m_y;
}

void RRTNode::setY(double y)
{
  m_y = y;
}

int RRTNode::getParent()
{
  return m_parent_index;
}

void RRTNode::setParent(int parent)
{
  m_parent_index = parent;
}

void RRTNode::setNode(double x, double y, int parent)
{
  m_x = x;
  m_y = y;
  m_parent_index = parent;
}
