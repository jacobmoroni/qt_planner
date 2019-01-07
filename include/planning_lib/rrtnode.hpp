#ifndef RRTNODE_HPP
#define RRTNODE_HPP


class RRTNode
{
public:
  RRTNode();
  RRTNode(double x, double y, int parent);
  double getX();
  void setX(double x);
  double getY();
  void setY(double y);
  int getParent();
  void setParent(int parent);
  void setNode(double x, double y, int parent);

private:
  double m_x{0};
  double m_y{0};
  int m_parent_index{-1};
};

#endif // RRTNODE_HPP
