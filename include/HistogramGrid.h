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

/**
 * The PolarHistogram is just an Eigen::VectorXi. It represents the likelihood
 * obstacle score in any particular direction from a given position of the 
 * robot.
 */
typedef Eigen::VectorXi PolarHistogram;


/**
 * @class HistogramGrid
 *
 * This histogram should be oriented so that the positive row direction is the 
 * same as the the robot's right (facing forward), and the positive column 
 * direction is the same as the robot's forward.
 *
 * Remember, in linear algebra, _rows are referenced first_.
 */
class HistogramGrid : public Eigen::MatrixXi
{

private:

  typedef Eigen::MatrixXi Base;

  unsigned int resolution_;  ///< The number of grid side lengths per meter
  // (e.g. resolution_ = 5 means 25 grid squares to a square meter)


public:

  // These are the methods that need to be defined for Eigen to work properly
  
  HistogramGrid() : Base(), resolution_(1)
  {   }

  HistogramGrid(
    unsigned int height,
    unsigned int width,
    unsigned int resolution = 1)
      : Base(height, width),
        resolution_(resolution)
  {   }

  //typedef Eigen::MatrixXi Base;

  // This constructor allows you to construct this class from Eigen expressions
  template<typename OtherDerived>
  HistogramGrid(
    const Eigen::MatrixBase<OtherDerived>& other, 
    unsigned int resolution = 1)
      : Base(other), resolution_(resolution)
  {   }

  // This method allows you to assign Eigen expressions to this class
  template<typename OtherDerived>
  HistogramGrid& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Base::operator=(other);
    return *this;
  }

  /**
   * @brief Returns the block centered at the coordinates with given height and 
   *        width.
   *
   * If `width` or `height` are even, they are incremented by one in the 
   * function. 
   * 
   * @param  robot_x The middle x-coord of the desired block
   * @param  robot_y The middle y-coord of the desired block
   * @param  width   The width of the block
   * @param  height  The height of the block
   * @return         A new sub-HistogramGrid
   */
  HistogramGrid getCenteredWindow(
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
    int robot_x, 
    int robot_y);

  /**
   * @brief Increments the obstacle likelihood count at the given index.
   * @param x The x-coord
   * @param y The y-coord
   */
  void increment(unsigned int x, unsigned int y);

  /**
   * @brief Returns the polar histogram for the robot's given coordinates. 
   * 
   * The histogram is oriented so that the 0 degrees points to the positive row
   * direction and 90 degrees points in the positive column direction.
   *
   * This is a little awkward to deal with in Eigen's initializer lists: 
   *
   * ```
   * Eigen::MatrixXi h;
   * h << 1, 2, 3, 4, 5,
   *      6, 7, 8, 9, 10,
   *      11,12,13,14,15,
   *      16,17,18,19,20,
   *      21,22,23,24,25;
   * ```
   * 
   * `h` as it appears above will be indexed by the polar histogram as follows:
   * 
   * ```
   * ------------> +y
   * | *  *  *  * 
   * | *  *  *  * 
   * | *  *  *  * ^ positive
   * | *  *  *  * | angle
   * V          __/
   * +x
   * ```
   * 
   * (+z comes out of the screen)
   * 
   * That being said, you have two choices for your layout of your obstacle 
   * matrix:
   * 1. Look at the comma initializer list sideways (with your head tilted
   *    to the right) and lay out your matrix as if the left edge of the 
   *    list (or the bottom, from your new angle) is the behind of the robot
   * 2. Look at it normally and then "rotate" the matrix values by 90 degrees
   *    clockwise in code by first transposing it and then reflecting it over 
   *    the y-axis, so that the robot (as you see it) has its back to the 
   *    bottom of your screen, and polar histogram will see things the way you
   *    see them, after a transformation. e.g. After the above initialization, 
   *    do this:
   * ```
   * h = h.transpose().eval();
   * for (int i = 0; i < h.cols()/2; ++i)
   * {
   *   h.col(i).swap(h.col(h.cols() - i - 1));
   * }
   * ```
   * 
   * @param  robot_x The robot's x-coord
   * @param  robot_y The robot's y-coord
   * @param  n       The number of desired sectors in the polar histogram.
   * @return         The polar histogram at `(robot_x, robot_y)` with `n` 
   *                 sectors.
   */
  PolarHistogram getPolarHistogram(double robot_x, double robot_y, unsigned int n) const;

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
