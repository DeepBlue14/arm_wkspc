#include <agile_grasp2/hand_search.h>


std::vector<GraspHypothesis> HandSearch::generateHypotheses(const CloudCamera& cloud_cam, int antipodal_mode,
  bool use_samples, bool forces_PSD, bool plots_normals, bool plots_samples)
{
  double t0_total = omp_get_wtime();

  // create KdTree for neighborhood search
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  cloud_normals_.resize(3, cloud->size());
  cloud_normals_.setZero(3, cloud->size());

  // 1. Calculate surface normals for all points (optional).
  bool has_normals = false;
  cloud_normals_ = cloud_cam.getNormals();
  if (plots_normals)
  {
    ROS_INFO("Plotting normals ...");
    plot_.plotNormals(cloud, cloud_normals_);
  }

  if (plots_samples)
  {
    ROS_INFO("Plotting samples ...");
    plot_.plotSamples(cloud_cam.getSampleIndices(), cloud_cam.getCloudProcessed());
  }

  // 2. Estimate local reference frames.
  std::cout << "Estimating local reference frames ...\n";
  std::vector<LocalFrame> frames;
  FrameEstimator frame_estimator(num_threads_);
  if (use_samples)
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSamples(), nn_radius_taubin_, kdtree);
  else
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSampleIndices(), nn_radius_taubin_, kdtree);
  if (plots_local_axes_)
    plot_.plotLocalAxes(frames, cloud_cam.getCloudOriginal());

  // 3. Evaluate possible hand placements.
  std::cout << "Finding hand poses ...\n";
  std::vector<GraspHypothesis> hypotheses = evaluateHandPlacements(cloud_cam, frames, nn_radius_hands_, kdtree);

  std::cout << "====> HAND SEARCH TIME: " << omp_get_wtime() - t0_total << std::endl;

  return hypotheses;
}


void HandSearch::evaluateGraspPoses(const CloudCamera& cloud_cam, std::vector<GraspHypothesis>& grasps, double radius)
{
  // create KdTree for neighborhood search
  const Eigen::MatrixXi& camera_source = cloud_cam.getCameraSource();
  const Eigen::Matrix3Xd& cloud_normals = cloud_cam.getNormals();
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::Matrix3Xd nn_normals;
  Eigen::MatrixXi nn_cam_source;
  Eigen::Matrix3Xd centered_neighborhood;
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_normals, nn_cam_source, centered_neighborhood) num_threads(num_threads_)
#endif
  for (int i = 0; i < grasps.size(); i++)
  {
    pcl::PointXYZRGBA sample = eigenVectorToPcl(grasps[i].getSample());

    if (kdtree.radiusSearch(sample, radius, nn_indices, nn_dists) > 0)
    {
      nn_normals = sliceMatrix(cloud_normals, nn_indices);
      centered_neighborhood = sliceMatrix(points, nn_indices);
      centered_neighborhood = centered_neighborhood - grasps[i].getSample().replicate(1, centered_neighborhood.cols());
      nn_cam_source = sliceMatrix(camera_source, nn_indices);

      evaluateHypothesis(centered_neighborhood, nn_normals, nn_cam_source, grasps[i]);
    }
  }
}


Eigen::Matrix3Xd HandSearch::sliceMatrix(const Eigen::Matrix3Xd& mat, const std::vector<int>& indices)
{
  Eigen::Matrix3Xd mat_out(3, indices.size());

  for (int j = 0; j < indices.size(); j++)
  {
    mat_out.col(j) = mat.col(indices[j]);
  }

  return mat_out;
}


Eigen::MatrixXi HandSearch::sliceMatrix(const Eigen::MatrixXi& mat, const std::vector<int>& indices)
{
  Eigen::MatrixXi mat_out(mat.rows(), indices.size());

  for (int j = 0; j < indices.size(); j++)
  {
    mat_out.col(j) = mat.col(indices[j]);
  }

  return mat_out;
}


pcl::PointXYZRGBA HandSearch::eigenVectorToPcl(const Eigen::Vector3d& v)
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}


void HandSearch::setParameters(const Parameters& params)
{
  finger_width_ = params.finger_width_;
  hand_outer_diameter_= params.hand_outer_diameter_;
  hand_depth_ = params.hand_depth_;
  hand_height_ = params.hand_height_;
  init_bite_ = params.init_bite_;

  num_threads_ = params.num_threads_;
  num_samples_ = params.num_samples_;
  nn_radius_taubin_ = params.nn_radius_taubin_;
  nn_radius_hands_ = params.nn_radius_hands_;
  num_orientations_ = params.num_orientations_;

  cam_tf_left_ = params.cam_tf_left_;
  cam_tf_right_ = params.cam_tf_right_;
}


std::vector<GraspHypothesis> HandSearch::evaluateHandPlacements(const CloudCamera& cloud_cam,
  const std::vector<LocalFrame>& frames, double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree)
{
  double t1 = omp_get_wtime();

  // possible angles describing hand orientations
  Eigen::VectorXd angles0 = Eigen::VectorXd::LinSpaced(num_orientations_ + 1, -1.0 * M_PI/2.0, M_PI/2.0);
  Eigen::VectorXd angles = angles0.block(0, 0, num_orientations_, 1);

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::Matrix3Xd nn_normals;
  Eigen::MatrixXi nn_cam_source;
  Eigen::Matrix3Xd nn_points_centered;
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  const Eigen::MatrixXi& camera_source = cloud_cam.getCameraSource();
  const Eigen::Matrix3Xd& normals = cloud_cam.getNormals();
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  std::vector< std::vector<GraspHypothesis> > hand_lists(frames.size(), std::vector<GraspHypothesis>(0));

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_points_centered, nn_normals, nn_cam_source) num_threads(num_threads_)
#endif
  for (std::size_t i = 0; i < frames.size(); i++)
  {
    pcl::PointXYZRGBA sample = eigenVectorToPcl(frames[i].getSample());

    if (kdtree.radiusSearch(sample, radius, nn_indices, nn_dists) > 0)
    {
      nn_normals = sliceMatrix(normals, nn_indices);
      nn_points_centered = sliceMatrix(points, nn_indices);
      nn_points_centered = nn_points_centered - frames[i].getSample().replicate(1, nn_points_centered.cols());
      nn_cam_source = sliceMatrix(camera_source, nn_indices);

      std::vector<GraspHypothesis> hands = calculateHypothesis(sample, nn_points_centered, nn_normals, nn_cam_source,
        frames[i], angles);
      if (hands.size() > 0)
        hand_lists[i] = hands;
    }
  }

  // concatenate the grasp lists
  std::vector<GraspHypothesis> hypotheses;
  for (std::size_t i = 0; i < hand_lists.size(); i++)
  {
    if (hand_lists[i].size() > 0)
      hypotheses.insert(hypotheses.end(), hand_lists[i].begin(), hand_lists[i].end());
  }

  double t2 = omp_get_wtime();
  std::cout << " Found " << hypotheses.size() << " robot hand poses in " << t2 - t1 << " sec.\n";

  return hypotheses;
}


std::vector<GraspHypothesis> HandSearch::calculateHypothesis(const pcl::PointXYZRGBA& sample, const Eigen::Matrix3Xd& points,
  const Eigen::Matrix3Xd& normals, const Eigen::MatrixXi& cam_source, const LocalFrame& local_frame,
  const Eigen::VectorXd& angles)
{
  FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

  // evaluate grasp at each hand orientation
  std::vector<GraspHypothesis> hand_list;
  for (int i = 0; i < angles.rows(); i++)
  {
    // rotate points into this hand orientation
    Eigen::Matrix3d rot;
    rot <<  cos(angles(i)),  -1.0 * sin(angles(i)),  0.0,
            sin(angles(i)),  cos(angles(i)),         0.0,
            0.0,             0.0,                    1.0;
//    rot <<  1.0,  0.0,             0.0,
//            0.0,  cos(angles(i)), -1.0 * sin(angles(i)),
//            0.0,  sin(angles(i)),  cos(angles(i));
    Eigen::Matrix3d rot_binormal;
    rot_binormal <<  -1.0,  0.0,  0.0,
                      0.0,  1.0,  0.0,
                      0.0,  0.0, -1.0;
    Eigen::Matrix3d local_frame_mat;
    local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();
    Eigen::Matrix3d frame_rot = local_frame_mat * rot_binormal * rot;
//    Eigen::Matrix3d frame_rot = local_frame_mat * rot;
    Eigen::Matrix3Xd points_rot = frame_rot.transpose() * points;
    Eigen::Matrix3Xd normals_rot = frame_rot.transpose() * normals;

    // crop points based on hand height
    Eigen::Matrix3Xd points_cropped;
    Eigen::Matrix3Xd normals_cropped;
    Eigen::MatrixXi cam_source_cropped;
    cropPointsAndNormals(points_rot, normals_rot, cam_source, hand_height_, points_cropped, normals_cropped, cam_source_cropped);
//    std::cout << "cam_source_cropped: " << cam_source_cropped.cols() << "\n";

    // evaluate finger locations for this orientation
    finger_hand.evaluateFingers(points_cropped, init_bite_);

    // check that there are at least two corresponding finger placements
    if (finger_hand.getFingers().cast<int>().sum() > 2)
    {
      finger_hand.evaluateHand();
//      std::cout << "hand: " << finger_hand.getHand() << "\n";

      if (finger_hand.getHand().cast<int>().sum() > 0)
      {
        // try to move the hand as deep as possible onto the object
        finger_hand.deepenHand(points_cropped, init_bite_, hand_depth_);

        // calculate points in the closing region of the hand
        std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(points_cropped);
        if (indices_learning.size() == 0)
        {
          std::cout << "#pts_rot: " << points.size() << ", #indices_learning: " << indices_learning.size() << "\n";
          continue;
        }

        GraspHypothesis hand = createGraspHypothesis(local_frame.getSample(), points_cropped, normals_cropped,
          cam_source_cropped, indices_learning, frame_rot, local_frame, finger_hand);
        hand_list.push_back(hand);
      }
    }
  }

  return hand_list;
}


void HandSearch::evaluateHypothesis(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& normals,
  const Eigen::MatrixXi& cam_source, GraspHypothesis& hand)
{
  FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

  // transform points into hand frame and crop them based on <hand_height>
  Eigen::Matrix3d frame = hand.getHandFrame();
  Eigen::Matrix3Xd points_frame = frame.transpose() * points;
  Eigen::Matrix3Xd normals_frame = frame.transpose() * normals;
  Eigen::Matrix3Xd points_cropped;
  Eigen::Matrix3Xd normals_cropped;
  Eigen::MatrixXi cam_source_cropped;
  cropPointsAndNormals(points_frame, normals_frame, cam_source, hand_height_, points_cropped, normals_cropped, cam_source_cropped);

  // evaluate finger locations for this orientation
  finger_hand.evaluateFingers(points_cropped, init_bite_);

  // check that there are at least two corresponding finger placements
  if (finger_hand.getFingers().cast<int>().sum() > 2)
  {
    finger_hand.evaluateHand();

    if (finger_hand.getHand().cast<int>().sum() > 0)
    {
      std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(points_cropped);
      if (indices_learning.size() == 0)
      {
        std::cout << "#pts_rot: " << points.size() << ", #indices_learning: " << indices_learning.size() << "\n";
        hand.setHalfAntipodal(false);
        hand.setFullAntipodal(false);
        return;
      }

      // extract data for classification
      Eigen::Matrix3Xd points_in_box(3, indices_learning.size());
      Eigen::Matrix3Xd normals_in_box(3, indices_learning.size());
      Eigen::MatrixXi cam_source_in_box(cam_source.rows(), indices_learning.size());
      for (int j = 0; j < indices_learning.size(); j++)
      {
        points_in_box.col(j) = points_cropped.col(indices_learning[j]);
        normals_in_box.col(j) = normals_cropped.col(indices_learning[j]);
        cam_source_in_box.col(j) = cam_source_cropped.col(indices_learning[j]);
      }
      double grasp_width = points_in_box.row(0).maxCoeff() - points_in_box.row(0).minCoeff();

      // evaluate if the grasp is antipodal
      Antipodal antipodal;
//      std::cout << "points_in_box: " << points_in_box.cols() << " normals_in_box: " << normals_in_box.cols() << std::endl;
      int antipodal_result = antipodal.evaluateGrasp(points_in_box, normals_in_box, 0.003);
//      std::cout << "antipodal_result: " << antipodal_result << std::endl;
      hand.setHalfAntipodal(antipodal_result == Antipodal::HALF_GRASP || antipodal_result == Antipodal::FULL_GRASP);
      hand.setFullAntipodal(antipodal_result == Antipodal::FULL_GRASP);
    }
  }
}


GraspHypothesis HandSearch::createGraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3Xd& points,
  const Eigen::Matrix3Xd& normals, const Eigen::MatrixXi& cam_source, const std::vector<int>& indices_learning,
  const Eigen::Matrix3d& hand_frame, const LocalFrame& local_frame, const FingerHand& finger_hand)
{
  Eigen::MatrixXd grasp_pos = finger_hand.calculateGraspParameters(hand_frame, sample);
  Eigen::Vector3d binormal = hand_frame.col(0);
  Eigen::Vector3d approach = hand_frame.col(1);
  Eigen::Vector3d axis = hand_frame.col(2);

  // extract data for classification
  Eigen::Matrix3Xd points_in_box = sliceMatrix(points, indices_learning);
  Eigen::Matrix3Xd normals_in_box = sliceMatrix(normals, indices_learning);
  Eigen::MatrixXi cam_source_in_box = sliceMatrix(cam_source, indices_learning);

  // calculate grasp width
  double grasp_width = points_in_box.row(0).maxCoeff() - points_in_box.row(0).minCoeff();

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  int antipodal_result = antipodal.evaluateGrasp(points_in_box, normals_in_box, 0.003);

  Eigen::Matrix3d frame;
  frame << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();
  GraspHypothesis hand(sample, frame, finger_hand, axis, approach, binormal, grasp_pos.col(0), grasp_pos.col(1),
    grasp_pos.col(2), grasp_width, points_in_box, normals_in_box, cam_source_in_box);
  hand.setHalfAntipodal(antipodal_result == Antipodal::HALF_GRASP || antipodal_result == Antipodal::FULL_GRASP);
  hand.setFullAntipodal(antipodal_result == Antipodal::FULL_GRASP);

  return hand;
}


void HandSearch::cropPointsAndNormals(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& normals,
  const Eigen::MatrixXi& cam_source, double height, Eigen::Matrix3Xd& points_out, Eigen::Matrix3Xd& normals_out,
  Eigen::MatrixXi& cam_source_out)
{
  std::vector<int> indices(points.cols());
  int k = 0;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(2, i) > -1.0 * height && points(2, i) < height)
    {
      indices[k] = i;
      k++;
    }
  }

  points_out.resize(3, k);
  normals_out.resize(3, k);
  cam_source_out.resize(cam_source.rows(), k);
  for (int i = 0; i < k; i++)
  {
    points_out.col(i) = points.col(indices[i]);
    normals_out.col(i) = normals.col(indices[i]);
    cam_source_out.col(i) = cam_source.col(indices[i]);
  }
}
