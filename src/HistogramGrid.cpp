

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
  // TODO: If the requested window is out-of-bounds, resize the block to 
  //  the requested size. This ensures that any obstacles that are detected
  //  that shouldn't be (such as walls) are thrown out.
  
  if (width % 2 == 0) ++width;
  if (height % 2 == 0) ++height;

  HistogramGrid ret(width, height, this->resolution_);
  ret << this->block(robot_x - width/2, robot_y - height/2, width, height);
  return ret;
}

void HistogramGrid::setCenteredWindow(
  const HistogramGrid& grid, 
  int robot_x,
  int robot_y)
{
  // check to make sure the indices are odd
  if (grid.rows() % 2 == 0 || grid.cols() % 2 == 0)
  {
    throw std::invalid_argument("This HistogramGrid has even bounds; "
      "it can't be set centered.");
  }

  // check to make sure the child grid at least overlaps with the parent
  if (grid.rows()/2 < -robot_x
    || grid.cols()/2 < -robot_y
    || robot_x - grid.rows()/2 > this->rows() - 1
    || robot_y - grid.cols()/2 > this->cols() - 1)
  {
    throw std::invalid_argument("This child HistogramGrid lies entirely "
      "outside of its parent.");
  }


  // To the guy who reads this: I'm so sorry. This wasn't part of the plan.
  // Eigen doesn't allow negative indices (like Python) or inferred sizes.
  // That's why this looks so bad. I swear I didn't do this on purpose.

  this->block(
    std::max(
      robot_x - grid.rows()/2, 
      static_cast<Eigen::Index>(0)
    ),
    std::max(
      robot_y - grid.cols()/2,
      static_cast<Eigen::Index>(0)
    ),
    std::min(
      this->rows() - 1,
      robot_x + grid.rows()/2
    ) - std::max(
      robot_x - grid.rows()/2,
      static_cast<Eigen::Index>(0)
    ) + static_cast<Eigen::Index>(1),
    std::min(
      this->cols() - 1,
      robot_y + grid.cols()/2
    ) - std::max(
      robot_x - grid.cols()/2,
      static_cast<Eigen::Index>(0)
    ) + static_cast<Eigen::Index>(1)
  ) = grid.block(
    std::max(
      grid.rows()/2 - robot_x,
      static_cast<Eigen::Index>(0)
    ),
    std::max(
      grid.cols()/2 - robot_y,
      static_cast<Eigen::Index>(0)
    ),
    std::min(
      this->rows() - 1,
      robot_x + grid.rows()/2
    ) - std::max(
      robot_x - grid.rows()/2,
      static_cast<Eigen::Index>(0)
    ) + static_cast<Eigen::Index>(1),
    std::min(
      this->cols() - 1,
      robot_y + grid.cols()/2
    ) - std::max(
      robot_x - grid.cols()/2,
      static_cast<Eigen::Index>(0)
    ) + static_cast<Eigen::Index>(1)
  );
}

void HistogramGrid::increment(unsigned int x, unsigned int y)
{
  ++this->operator()(x,y);
}

PolarHistogram HistogramGrid::getPolarHistogram(
  double robot_x, 
  double robot_y, 
  unsigned int n) const
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
        p[
          static_cast<int>(
            std::floor(
              beta * static_cast<double>(n) / (2.0 * M_PI)
            )
          )
        ] += this->operator()(i,j);
      }
    }
  }

  return p;
}

#endif //HISTOGRAM_GRID_CPP_
