#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include <boost/lexical_cast.hpp>

#include <agile_grasp2/antipodal.h>


Eigen::Matrix3Xd fillMatrixFromFile(const std::string& filename)
{
  std::ifstream in;
  in.open(filename.c_str());

  std::string line;
  Eigen::Matrix3Xd mat(3, 1319);
  int i = 0;

  while(std::getline(in, line))
  {
    std::stringstream  lineStream(line);
    std::string        cell;
    int j = 0;

    while(std::getline(lineStream, cell, ','))
    {
      std::cout << cell << std::endl;
      mat(i,j) = boost::lexical_cast<double>(cell);
      j++;
    }

    i++;
  }

  return mat;
}


int main()
{
   std::ifstream in;
   in.open("/home/andreas/pts.csv");

   std::string line;
   Eigen::Matrix3Xd pts = fillMatrixFromFile("/home/andreas/pts.csv");
   Eigen::Matrix3Xd normals = fillMatrixFromFile("/home/andreas/normals.csv");

   Antipodal anti;
   anti.evaluateGrasp(pts, normals, 0.003);

   return 0;
}
