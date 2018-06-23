#include <agile_grasp2/handle_search.h>


std::vector<GraspHypothesis> HandleSearch::findClusters(const std::vector<GraspHypothesis>& hand_list,
  bool remove_inliers)
{
  const double AXIS_ALIGN_ANGLE_THRESH = 15.0 * M_PI/180.0;
  const double AXIS_ALIGN_DIST_THRESH = 0.005;
  const double MAX_DIST_THRESH = 0.07;
  //  const int max_inliers = 50;

  std::vector<GraspHypothesis> hands_out;
  std::vector<bool> has_used;
  if (remove_inliers)
  {
    has_used.resize(hand_list.size());
    for (int i = 0; i < hand_list.size(); i++)
    {
      has_used[i] = false;
    }
  }

  std::vector<int> inliers;

  for (int i = 0; i < hand_list.size(); i++)
  {
    int num_inliers = 0;
    Eigen::Vector3d grasp_pos_delta = Eigen::Vector3d::Zero();
    Eigen::Matrix3d axis_outer_prod = hand_list[i].getAxis() * hand_list[i].getAxis().transpose();
    inliers.resize(0);
    double avg_score = 0.0;

    for (int j = 0; j < hand_list.size(); j++)
    {
      if (i == j || (remove_inliers && has_used[j]))
        continue;

      // Which hands have an axis within <AXIS_ALIGN_ANGLE_THRESH> of this one?
      double axis_aligned = hand_list[i].getAxis().transpose() * hand_list[j].getAxis();
      bool axis_aligned_binary = fabs(axis_aligned) > cos(AXIS_ALIGN_ANGLE_THRESH);

      // Which hands are within <MAX_DIST_THRESH> of this one?
      Eigen::Vector3d delta_pos = hand_list[i].getGraspBottom() - hand_list[j].getGraspBottom();
      double delta_pos_mag = delta_pos.norm();
      bool delta_pos_mag_binary = delta_pos_mag <= MAX_DIST_THRESH;

      // Which hands are within <AXIS_ALIGN_DIST_THRESH> of this one when projected onto the plane orthognal to this
      // one's axis?
      Eigen::Matrix3d axis_orth_proj = Eigen::Matrix3d::Identity() - axis_outer_prod;
      Eigen::Vector3d delta_pos_proj = axis_orth_proj * delta_pos;
      double delta_pos_proj_mag = delta_pos_proj.norm();
      bool delta_pos_proj_mag_binary = delta_pos_proj_mag <= AXIS_ALIGN_DIST_THRESH;

      bool inlier_binary = axis_aligned_binary && delta_pos_mag_binary && delta_pos_proj_mag_binary;
      if (inlier_binary)
      {
        inliers.push_back(i);
        num_inliers++;
        grasp_pos_delta += hand_list[j].getGraspBottom();
        avg_score += hand_list[j].getScore();
        if (remove_inliers)
          has_used[j] = true;
        //        if (num_inliers >= max_inliers)
        //          break;
      }
    }

    if (num_inliers >= min_inliers_)
    {
      grasp_pos_delta = grasp_pos_delta / (double) num_inliers - hand_list[i].getGraspBottom();
      avg_score = avg_score / (double) num_inliers;
      std::cout << "grasp " << i << ", num_inliers: " << num_inliers << ", pos_delta: " << grasp_pos_delta.transpose()
          << ", avg score: " << avg_score << "\n";
      GraspHypothesis hand = hand_list[i];
      hand.setGraspSurface(hand.getGraspSurface() + grasp_pos_delta);
      hand.setGraspBottom(hand.getGraspBottom() + grasp_pos_delta);
      hand.setGraspTop(hand.getGraspTop() + grasp_pos_delta);
      hand.setScore(avg_score);
      hands_out.push_back(hand);
    }
  }

  return hands_out;
}
