

#include "HistogramGrid.h"

#include <iostream>


void flipx(Eigen::MatrixXi &m)
{
  for (int i = 0; i < m.rows()/2; ++i)
  {
    m.row(i).swap(m.row(m.rows() - i - 1));
  }
}

void flipy(Eigen::MatrixXi &m)
{
  for (int i = 0; i < m.cols()/2; ++i)
  {
    m.col(i).swap(m.col(m.cols() - i - 1));
  }
}

int main(void)
{
  HistogramGrid h(5, 5, 5);
  // h << 1, 2, 3, 4, 5,
  //      6, 7, 8, 9, 10,
  //      11,12,13,14,15,
  //      16,17,18,19,20,
  //      21,22,23,24,25;
  h << 0,3,0,2,0,
       4,0,0,0,1,
       0,0,1,0,0,
       5,0,0,0,8,
       0,6,0,7,0;


  // READER, TAKE NOTE!!
  // Read the docs on the getPolarHistogram() method to figure out what I'm 
  // doing here.
  h = h.transpose().eval();
  flipy(h);

  HistogramGrid h2 = h.getCenteredWindow(2, 2, 3, 3);
  // h2.increment(0, 0);
  // h2.increment(0, 0);
  // h2.increment(0, 1);
  // h2.increment(0, 2);
  // h2.increment(0, 2);
  h.setCenteredWindow(h2, 2, 2);

  PolarHistogram p = h.getPolarHistogram(0.0, 0.0, 8);

  //h.block(1,1,3,3) << 0,0,0,0;
  
  std::cout << "h = " << std::endl
            << h << std::endl;

  std::cout << "p = " << std::endl << p.transpose() << std::endl;

  std::cout << "It compiles...!" << std::endl;
  return 0;
}