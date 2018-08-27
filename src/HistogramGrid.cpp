

#ifndef HISTOGRAM_GRID_CPP_
#define HISTOGRAM_GRID_CPP_

#include "HistogramGrid.h"

#include <cmath>
#ifndef M_PI
  #define M_PI 3.14159265358979323846264338327950288
#endif



HistogramGrid HistogramGrid::getCenteredWindow(
  unsigned int robot_x,
  unsigned int robot_y,
  unsigned int width,
  unsigned int height) const
{
  if (width % 2 == 0) ++width;
  if (height % 2 == 0) ++height;

  HistogramGrid ret(width, height);
  ret << this->block(robot_x - width/2, robot_y - height/2, width, height);
  return ret;
}

void HistogramGrid::setCenteredWindow(
  const HistogramGrid& grid, 
  unsigned int robot_x,
  unsigned int robot_y)
{
  if (grid.rows() % 2 == 0 || grid.cols() % 2 == 0)
    throw std::invalid_argument("This HistogramGrid has even bounds; "
      "it can't be set centered.");

  this->block(
    robot_x - grid.rows()/2, 
    robot_y - grid.cols()/2, 
    grid.rows(), 
    grid.cols())
      = grid;
}

void HistogramGrid::increment(unsigned int x, unsigned int y)
{
  ++this->operator()(x,y);
}

PolarHistogram HistogramGrid::getPolarHistogram(
  double robot_x, 
  double robot_y, 
  int n) const
{
  double alpha = 360.0/static_cast<double>(n);
  PolarHistogram p(n);

  for (int i = 0; i < this->cols(); ++i) //row
  {
    for (int j = 0; j < this->rows(); ++j) //column
    {
      // if the grid square has values, add them to the p-hist
      if (this->operator()(i,j) != 0)
      {
        double beta = std::atan2(
          static_cast<double>(j) + 0.5 - robot_y,
          static_cast<double>(i) + 0.5 - robot_x);
        if (beta < 0)
          beta += 2.0*M_PI;

        // increment the given angle
        p[static_cast<int>(std::floor(beta*static_cast<double>(n)/(2.0*M_PI)))]
          += this->operator()(i,j);
      }
    }
  }

  return p;
}

#endif //HISTOGRAM_GRID_CPP_
