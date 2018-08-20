/**
 * @file HistogramGrid.h
 *
 * The histogram grid, which records the number of times an obstacle is 
 * detected within a designated area. 
 */

#ifndef HISTOGRAM_GRID_H_
#define HISTOGRAM_GRID_H_

#include <Eigen/Dense>
#include <iostream>


class HistogramGrid : Eigen::MatrixXi
{

private:

  unsigned int resolution_;  ///< The number of grid side lengths per meter
  // (e.g. resolution_ = 5 means 25 grid squares to a square meter)


public:

  // These are the methods that need to be defined for Eigen to work properly
  
  HistogramGrid() : Eigen::MatrixXi(), resolution_(1)
  {   }

  HistogramGrid(
    unsigned int height,
    unsigned int width,
    unsigned int resolution = 1)
      : Eigen::MatrixXi(height, width),
        resolution_(resolution)
  {   }

  //typedef Eigen::MatrixXi Base;

  // This constructor allows you to construct this class from Eigen expressions
  template<typename OtherDerived>
  HistogramGrid(
    const Eigen::MatrixBase<OtherDerived>& other, 
    unsigned int resolution = 1)
      : Eigen::MatrixXi(other), resolution_(resolution)
  {   }

  // This method allows you to assign Eigen expressions to this class
  template<typename OtherDerived>
  HistogramGrid& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::MatrixXi::operator=(other);
    return *this;
  }

  /**
   * @brief Returns the block centered at the coordinates with given height and 
   *        width.
   *
   * If `width` or `height` are even, this will return a block with width and 
   * height `width + 1` and `height + 1`, respectively.
   *
   * This function returns an lvalue, just like in Eigen's `block()` function.
   * 
   * @param  robot_x The middle x-coord of the desired block
   * @param  robot_y The middle y-coord of the desired block
   * @param  width   The width of the block
   * @param  height  The height of the block
   * @return         The desired lvalue block
   */
  HistogramGrid& cblock(
    unsigned int robot_x, 
    unsigned int robot_y, 
    unsigned int width, 
    unsigned int height);

  /**
   * [cblock description]
   * @param  robot_x [description]
   * @param  robot_y [description]
   * @param  width   [description]
   * @param  height  [description]
   * @return         [description]
   */
  const HistogramGrid& cblock(
    unsigned int robot_x, 
    unsigned int robot_y, 
    unsigned int width, 
    unsigned int height) const;

  /**
   * [getCenteredWindow description]
   * @param  robot_x [description]
   * @param  robot_y [description]
   * @param  width   [description]
   * @param  height  [description]
   * @return         [description]
   * @see cblock
   */
  HistogramGrid& getCenteredWindow(
    unsigned int robot_x, 
    unsigned int robot_y, 
    unsigned int width, 
    unsigned int height);

  /**
   * [getCenteredWindow description]
   * @param  robot_x [description]
   * @param  robot_y [description]
   * @param  width   [description]
   * @param  height  [description]
   * @return         [description]
   * @see cblock
   */
  const HistogramGrid& getCenteredWindow(
    unsigned int robot_x, 
    unsigned int robot_y, 
    unsigned int width, 
    unsigned int height) const;

  /**
   * @brief Sets the values in this HistogramGrid to the centered values in the 
   *        passed grid
   * 
   * @param grid    The HistogramGrid to set this HistogramGrid's values to
   * @param robot_x The x-coord of the center of the grid
   * @param robot_y The y-coord of the center of the grid
   *
   * @throws std::invalid_argument if grid has an even number of columns or rows
   */
  void setCenteredWindow(
    const HistogramGrid& grid, 
    unsigned int robot_x, 
    unsigned int robot_y);

  /**
   * [increment description]
   * @param x [description]
   * @param y [description]
   */
  void increment(unsigned int x, unsigned int y);

  /* *
   * [getSectionCount description]
   * @param  robot_x [description]
   * @param  robot_y [description]
   * @param  alpha   [description]
   * @param  n       [description]
   * @return         [description]
   */
  // unsigned int getSectorCount(
  //   unsigned int robot_x, 
  //   unsigned int robot_y,
  //   unsigned int alpha,
  //   unsigned int n);

  /* *
   * [getSectorCount description]
   * @param  robot_x [description]
   * @param  robot_y [description]
   * @param  angle1  [description]
   * @param  angle2  [description]
   * @return         [description]
   */
  // unsigned int getSectorCount(
  //   unsigned int robot_x,
  //   unsigned int robot_y,
  //   unsigned double angle1,
  //   unsigned double angle2);

};


#endif //HISTOGRAM_GRID_H_
