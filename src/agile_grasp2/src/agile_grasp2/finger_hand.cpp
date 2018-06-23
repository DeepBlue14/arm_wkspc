#include <agile_grasp2/finger_hand.h>


FingerHand::FingerHand(double finger_width, double hand_outer_diameter,	double hand_depth)
  :	finger_width_(finger_width), hand_outer_diameter_(hand_outer_diameter), hand_depth_(hand_depth)
{
	int n = 10; // number of finger placements to consider over a single hand diameter

	Eigen::VectorXd fs_half;
	fs_half.setLinSpaced(n, 0.0, hand_outer_diameter - finger_width);
	finger_spacing_.resize(2 * fs_half.size());
	finger_spacing_	<< (fs_half.array() - hand_outer_diameter_ + finger_width_).matrix(), fs_half;
	fingers_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, 2 * n, false);
}


void FingerHand::evaluateFingers(const Eigen::Matrix3Xd& points, double bite, int idx)
{
  // calculate fingertip distance (top) and hand base distance (bottom)
  top_ = bite;
  bottom_ = bite - hand_depth_;

  fingers_.setConstant(false);

  // crop points at bite
  std::vector<int> cropped_indices;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(1, i) < bite)
    {
      cropped_indices.push_back(i);

      // Check that the hand would be able to extend by <bite> onto the object without causing the back of the hand to
      // collide with <points>.
      if (points(1, i) < bottom_)
      {
//        std::cout << "back of hand collides with points!\n";
//        std::cout << "  bottom: " << bottom_ << ", top: " << top_ << "\n";
        return;
      }
    }
  }

  // check that there is at least one point in between the fingers
  if (cropped_indices.size() == 0)
    return;

  Eigen::MatrixXd cropped_points(points.rows(), cropped_indices.size());
  for (int i = 0; i < cropped_indices.size(); i++)
  {
    cropped_points.col(i) = points.col(cropped_indices[i]);
  }

  // identify free gaps
  int m = finger_spacing_.size();
  if (idx == -1)
  {
    for (int i = 0; i < m; i++)
    {
//      int num_in_gap = (cropped_points.row(0).array() > finger_spacing_(i)
//                        && cropped_points.row(0).array() < finger_spacing_(i) + finger_width_).count();
      int num_in_gap = 0;
      for (int j = 0; j < cropped_indices.size(); j++)
      {
        if (cropped_points(0, j) > finger_spacing_(i) && cropped_points(0, j) < finger_spacing_(i) + finger_width_)
          num_in_gap++;
      }

//      std::cout << "num_in_gap: " << num_in_gap << std::endl;

      if (num_in_gap == 0)
      {
        fingers_(i) = true;
      }
    }
  }
  else
  {
    for (int i = 0; i < m; i++)
    {
      if (i == idx || i == m/2 + idx)
      {
        int num_in_gap = 0;
        for (int j = 0; j < cropped_indices.size(); j++)
        {
          if (cropped_points(0, j) > finger_spacing_(i) && cropped_points(0, j) < finger_spacing_(i) + finger_width_)
            num_in_gap++;
        }

        if (num_in_gap == 0)
        {
          fingers_(i) = true;
        }
      }
    }
  }

//  std::cout << "evalGrasps() -- fingers_: " << fingers_ << "\n";
}


void FingerHand::deepenHand(const Eigen::Matrix3Xd& points, double min_depth, double max_depth)
{
  std::vector<int> hand_idx;

  for (int i = 0; i < hand_.cols(); i++)
  {
    if (hand_(i) == true)
      hand_idx.push_back(i);
  }

  if (hand_idx.size() == 0)
    return;

  // choose middle hand
  int hand_eroded_idx = hand_idx[ceil(hand_idx.size() / 2.0) - 1]; // middle index
  Eigen::Array<bool, 1, Eigen::Dynamic> hand_eroded;
  hand_eroded = Eigen::Array<bool, 1,Eigen::Dynamic>::Constant(hand_.cols(), false);
  hand_eroded(hand_eroded_idx) = true;

  // attempt to deepen hand
  double deepen_step_size = 0.005;
  FingerHand new_hand = *this;
  FingerHand last_new_hand = new_hand;

  for (double depth = min_depth + deepen_step_size; depth <= max_depth; depth += deepen_step_size)
  {
    new_hand.evaluateFingers(points, depth, hand_eroded_idx);
    if (new_hand.getFingers().cast<int>().sum() < 2)
      break;

    new_hand.evaluateHand();
    last_new_hand = new_hand;
  }

  *this = last_new_hand; // recover the deepest hand
  hand_ = hand_eroded;
}


std::vector<int> FingerHand::computePointsInClosingRegion(const Eigen::Matrix3Xd& points)
{
  // find feasible hand location
  int idx = -1;
  for (int i = 0; i < hand_.cols(); i++)
  {
    if (hand_(i) == true)
    {
      idx = i;
      break;
    }
  }
  if (idx == -1)
  {
    std::cout << "ERROR: Something went wrong!\n";
  }

//  std::cout << "  finger_spacing: " << finger_spacing_.transpose() << "\n";

  // calculate the horizontal parameters of the closing region of the robot hand for this finger placement
  left_ = finger_spacing_(idx) + finger_width_;
  right_ = finger_spacing_(hand_.cols() + idx);
  center_ = 0.5 * (left_ + right_);
//  surface_ = points.row(1).minCoeff();
  surface_ = points.row(0).minCoeff();
//  std::cout << finger_spacing_.transpose() << "\n";
//  std::cout << " left_: " << left_ << ", right_: " << right_ << "\n";
//  std::cout << " center_: " << center_ << ", surface_: " << surface_ << "\n";
//  std::cout << " bottom_: " << bottom_ << ", top_: " << top_ << "\n";

  // find points inside closing region defined by <bottom_>, <top_>, <left_> and <right_>
  std::vector<int> indices;
  for (int i = 0; i < points.cols(); i++)
  {
//    std::cout << i << ": " << points.col(i).transpose() << "\n";

    if (points(1, i) > bottom_ && points(1, i) < top_ && points(0, i) > left_ && points(0, i) < right_)
      indices.push_back(i);

//    if (points(0, i) > bottom_ && points(0, i) < top_ && points(1, i) > left_ && points(1, i) < right_)
//      indices.push_back(i);
  }

  return indices;
}


Eigen::MatrixXd FingerHand::calculateGraspParameters(const Eigen::Matrix3d& frame, const Eigen::Vector3d& sample) const
{
  Eigen::MatrixXd params(3,7);

  Eigen::Vector3d pos_surface, pos_bottom, pos_top, left_bottom, right_bottom, left_top, right_top;

  // calculate position of hand middle on object surface
  pos_surface << center_, surface_, 0.0;
  params.col(0) = frame * pos_surface + sample;

  // calculate position of hand middle closest to robot hand base
  pos_bottom << center_, bottom_, 0.0;
  params.col(1) = frame * pos_bottom + sample;

  // calculate position of hand middle between fingertips
  pos_top << center_, top_, 0.0;
  params.col(2) = frame * pos_top + sample;

  // calculate bottom and top positions for left and right finger
  left_bottom << left_, bottom_, 0.0;
  right_bottom << right_, bottom_, 0.0;
  left_top << left_, top_, 0.0;
  right_top << right_, top_, 0.0;
  params.col(3) = frame * left_bottom + sample;
  params.col(4) = frame * right_bottom + sample;
  params.col(5) = frame * left_top + sample;
  params.col(6) = frame * right_top + sample;

//  std::cout << "center_: " << center_ << ", surface_: " << surface_ << ", bottom_: " << bottom_ << ", top_: " << top_ << std::endl;

  return params;
}


void FingerHand::evaluateHand()
{
	int n = fingers_.size() / 2;
	hand_.resize(1, n);

	for (int i = 0; i < n; i++)
	{
		if (fingers_(i) == true && fingers_(n + i) == true)
			hand_(i) = 1;
		else
			hand_(i) = 0;
	}
}
