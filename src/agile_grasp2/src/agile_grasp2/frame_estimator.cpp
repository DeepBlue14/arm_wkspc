#include <agile_grasp2/frame_estimator.h>


std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(const CloudCamera& cloud_cam,
  const std::vector<int>& indices, double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree)
{
  double t1 = omp_get_wtime();
  std::vector<LocalFrame*> frames(indices.size());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < indices.size(); i++)
  {
    const pcl::PointXYZRGBA& sample = cloud_cam.getCloudProcessed()->points[indices[i]];
    LocalFrame* frame = calculateLocalFrame(cloud_cam, sample.getVector3fMap().cast<double>(), radius, kdtree);
    frames[i] = frame;
//    frames[i]->print();
  }

  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++)
  {
    if (frames[i])
      frames_out.push_back(*frames[i]);
    delete frames[i];
  }
  frames.clear();

  double t2 = omp_get_wtime();
  std::cout << "Fitted " << frames_out.size() << " local reference frames in " << t2 - t1 << " sec.\n";

  return frames_out;
}


std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(const CloudCamera& cloud_cam, const Eigen::Matrix3Xd& samples,
  double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree)
{
  double t1 = omp_get_wtime();
  std::vector<LocalFrame*> frames(samples.cols());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < samples.cols(); i++)
  {
    pcl::PointXYZRGBA sample = eigenVectorToPcl(samples.col(i));

    LocalFrame* frame = calculateLocalFrame(cloud_cam, samples.col(i), radius, kdtree);
    frames[i] = frame;
    //    frames[i]->print();
  }

  // delete empty frames
  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++)
  {
    if (frames[i])
      frames_out.push_back(*frames[i]);
    delete frames[i];
  }
  frames.clear();

  double t2 = omp_get_wtime();
  std::cout << "Fitted " << frames_out.size() << " local reference frames in " << t2 - t1 << " sec.\n";

  return frames_out;
}


LocalFrame* FrameEstimator::calculateLocalFrame(const CloudCamera& cloud_cam, const Eigen::Vector3d& sample, double radius,
  const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree)
{
  LocalFrame* frame = NULL;
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::VectorXi num_source(cloud_cam.getCameraSource().rows());
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  const Eigen::Matrix3Xd& normals = cloud_cam.getNormals();
  const Eigen::MatrixXi& camera_source = cloud_cam.getCameraSource();
  pcl::PointXYZRGBA sample_pcl = eigenVectorToPcl(sample);

  if (kdtree.radiusSearch(sample_pcl, radius, nn_indices, nn_dists) > 0)
  {
    int nn_num_samples = 50;
    Eigen::Matrix3Xd nn_normals(3, std::min(nn_num_samples, (int) nn_indices.size()));
    num_source.setZero();

    for (int j = 0; j < nn_normals.cols(); j++)
    {
      int r = rand() % nn_indices.size();
      while (isnan(cloud->points[nn_indices[r]].x))
      {
        r = rand() % nn_indices.size();
      }
      nn_normals.col(j) = normals.col(nn_indices[r]);

      for (int cam_idx = 0; cam_idx < camera_source.rows(); cam_idx++)
      {
        if (camera_source(cam_idx, nn_indices[r]) == 1)
          num_source(cam_idx)++;
      }
    }

    // calculate camera source for majority of points
    int majority_cam_source;
    num_source.maxCoeff(&majority_cam_source);

    Eigen::MatrixXd gradient_magnitude = ((nn_normals.cwiseProduct(nn_normals)).colwise().sum()).cwiseSqrt();
    nn_normals = nn_normals.cwiseQuotient(gradient_magnitude.replicate(3, 1));
    frame = new LocalFrame(sample, majority_cam_source);
    frame->findAverageNormalAxis(nn_normals);
  }

  return frame;
}


pcl::PointXYZRGBA FrameEstimator::eigenVectorToPcl(const Eigen::Vector3d& v)
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}
