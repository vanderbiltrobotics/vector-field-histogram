

#ifndef HISTOGRAM_GRID_CPP_
#define HISTOGRAM_GRID_CPP_

#include "HistogramGrid.h"

HistogramGrid& HistogramGrid::cblock(
  unsigned int robot_x, 
  unsigned int robot_y, 
  unsigned int width, 
  unsigned int height)
{
  if (width % 2 == 0) ++width;
  if (height % 2 == 0) ++height;

  return this->block(robot_x - width/2, robot_y - height/2, width, height);
}

const HistogramGrid& HistogramGrid::cblock(
  unsigned int robot_x, 
  unsigned int robot_y, 
  unsigned int width, 
  unsigned int height) const
{
  return this->cblock(robot_x, robot_y, width, height);
}

HistogramGrid& HistogramGrid::getCenteredWindow(
  unsigned int robot_x, 
  unsigned int robot_y, 
  unsigned int width, 
  unsigned int height)
{
  return this->cblock(robot_x, robot_y, width, height);
}

const HistogramGrid& HistogramGrid::getCenteredWindow(
  unsigned int robot_x, 
  unsigned int robot_y, 
  unsigned int width, 
  unsigned int height) const
{
  return this->cblock(robot_x, robot_y, width, height);
}

void HistogramGrid::setCenteredWindow(
  const HistogramGrid& grid, 
  unsigned int robot_x, 
  unsigned int robot_y)
{
  if (grid.rows() % 2 == 0 || grid.cols() % 2 == 0)
    throw std::invalid_argument("This HistogramGrid has even bounds; "
      "it can't be set centered.");

  this->cblock(robot_x, robot_y, grid.rows(), grid.cols()) = grid;
}

void HistogramGrid::increment(unsigned int x, unsigned int y)
{
  ++this->operator()(x,y);
}

#endif //HISTOGRAM_GRID_CPP_
