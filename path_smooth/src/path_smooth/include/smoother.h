/**
 * @file smoother.h
 * @brief 用于将路径进行平滑处理的类
 *  
 */

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>
#include <nav_msgs/Path.h>

#include "vector2d.h"


class Smoother {
 public:
  Smoother() {}
  nav_msgs::Path smoothPath(const nav_msgs::Path::ConstPtr& path);
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

 private:
  float alpha = 0.0;
  float wSmoothness = 0.2;
};
#endif // SMOOTHER_H
