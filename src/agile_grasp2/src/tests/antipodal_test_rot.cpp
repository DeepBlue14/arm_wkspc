#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include <boost/lexical_cast.hpp>

#include <agile_grasp2/antipodal.h>
#include <agile_grasp2/finger_hand.h>


Eigen::Matrix3Xd fillMatrixFromFile(const std::string& filename)
{
  std::ifstream in;
  in.open(filename.c_str());

  std::string line;
  Eigen::Matrix3Xd mat(3, 5211);
  int i = 0;

  while(std::getline(in, line))
  {
    std::stringstream  lineStream(line);
    std::string        cell;
    int j = 0;

    while(std::getline(lineStream, cell, ','))
    {
      mat(i,j) = boost::lexical_cast<double>(cell);
      j++;
    }

    i++;
  }

  return mat;
}


int main()
{
  std::string line;
  Eigen::Matrix3Xd points = fillMatrixFromFile("/home/andreas/pts1.csv");
  std::cout << "loaded points\n";
  Eigen::Matrix3Xd normals = fillMatrixFromFile("/home/andreas/normals1.csv");
  std::cout << "loaded normals\n";

  double finger_width_ = 0.01;
  double hand_outer_diameter_ = 0.12;
  double hand_depth_ = 0.06;
  double hand_height_ = 0.02;
  double init_bite_ = 0.015;

  FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

  // extract axes of local reference frame
  Eigen::Matrix3d frame;
  frame << -0.5031,    0.8575,   -0.1075,
            0.8641,    0.4972,   -0.0779,
           -0.0133,   -0.1320,   -0.9912;
  Eigen::Vector3d sample;
  sample << 0.0502, -0.0576, -0.0031;

  // transform points into local reference frame and crop them based on <hand_height>
  Eigen::Matrix3Xd points_frame = frame.transpose() * (points - sample.replicate(1, points.cols()));
  Eigen::Matrix3Xd normals_frame = frame.transpose() * normals;
  std::vector<int> indices(points_frame.cols());
  std::cout << points_frame.block<3,4>(0,0) << "\n";
  std::cout << normals_frame.block<3,4>(0,0) << "\n";
  int k = 0;
  for (int i = 0; i < points_frame.cols(); i++)
  {
    if (points_frame(2, i) > -1.0 * hand_height_ && points_frame(2, i) < hand_height_)
    {
      indices[k] = i;
//      std::cout << k << ", " << indices[k] << "\n";
      k++;
    }
  }
  std::cout << "k: " << k << "\n";
  Eigen::Matrix3Xd points_cropped(3, k);
  Eigen::Matrix3Xd normals_cropped(3, k);
//  Eigen::MatrixXi cam_source_cropped(cam_source.rows(), k);
  for (int i = 0; i < k; i++)
  {
    points_cropped.col(i) = points_frame.col(indices[i]);
    normals_cropped.col(i) = normals_frame.col(indices[i]);
//    cam_source_cropped.col(i) = cam_source.col(indices[i]);
  }

  finger_hand.evaluateFingers(points_cropped, init_bite_);
  finger_hand.evaluateHand();
  std::cout << "(before deepening) hand: " << finger_hand.getHand() << "\n\n";
  finger_hand.deepenHand(points_cropped, init_bite_, hand_depth_);
  std::cout << "(after) hand: " << finger_hand.getHand() << "\n";

  // extract data for classification
  std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(points_cropped);
  std::cout << "indices_learning: " << indices_learning.size() << "\n";
  Eigen::Matrix3Xd points_in_box(3, indices_learning.size());
  Eigen::Matrix3Xd normals_in_box(3, indices_learning.size());
//  Eigen::MatrixXi cam_source_in_box(cam_source.rows(), indices_learning.size());
  for (int j = 0; j < indices_learning.size(); j++)
  {
    points_in_box.col(j) = points_cropped.col(indices_learning[j]);
    normals_in_box.col(j) = normals_cropped.col(indices_learning[j]);
//    cam_source_in_box.col(j) = cam_source.col(indices_learning[j]);
  }

  Antipodal anti;
  anti.evaluateGrasp(points_in_box, normals_in_box, 0.003);

  return 0;
}
