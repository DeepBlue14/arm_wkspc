#include <agile_grasp2/importance_sampling.h>


// methods for sampling from a set of Gaussians
const int SUM_OF_GAUSSIANS = 0;
const int MAX_OF_GAUSSIANS = 1;

// standard parameters
const int ImportanceSampling::NUM_ITERATIONS = 5;
const int ImportanceSampling::NUM_SAMPLES = 50;
const int ImportanceSampling::NUM_INIT_SAMPLES = 50;
const double ImportanceSampling::PROB_RAND_SAMPLES = 0.3;
const double ImportanceSampling::RADIUS = 0.02;
const bool ImportanceSampling::VISUALIZE_STEPS = false;
const bool ImportanceSampling::VISUALIZE_RESULTS = true;
const int ImportanceSampling::SAMPLING_METHOD = SUM_OF_GAUSSIANS;


ImportanceSampling::ImportanceSampling(ros::NodeHandle& node) : GraspDetector(node)
{
  node.param("num_init_samples", num_init_samples_, NUM_INIT_SAMPLES);
  node.param("num_iterations", num_iterations_, NUM_ITERATIONS);
  node.param("num_samples_per_iteration", num_samples_, NUM_SAMPLES);
  node.param("prob_rand_samples", prob_rand_samples_, PROB_RAND_SAMPLES);
  node.param("std", radius_, RADIUS);
  node.param("sampling_method", sampling_method_, SAMPLING_METHOD);
  node.param("visualize_steps", visualize_steps_, VISUALIZE_STEPS);
  node.param("visualize_results", visualize_results_, VISUALIZE_RESULTS);
}


std::vector<GraspHypothesis> ImportanceSampling::detectGraspPoses(const CloudCamera& cloud_cam_in)
{
  double t0 = omp_get_wtime();
  CloudCamera cloud_cam = cloud_cam_in;
  Plot plotter;
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();

  // 1. Find initial grasp hypotheses.
  std::vector<GraspHypothesis> hands = GraspDetector::detectGraspPoses(cloud_cam, false);
  std::cout << "Initially detected " << hands.size() << " grasp hypotheses" << std::endl;
  if (hands.size() == 0)
  {
    return hands;
  }
  if (visualize_steps_)
  {
    plotter.plotFingers(hands, cloud_cam.getCloudOriginal(), "Initial Grasps");
  }

  // 2. Create random generator for normal distribution.
  int num_rand_samples = prob_rand_samples_ * num_samples_;
  int num_gauss_samples = num_samples_ - num_rand_samples;
  double sigma = radius_;
  Eigen::Matrix3d diag_sigma = Eigen::Matrix3d::Zero();
  diag_sigma.diagonal() << sigma, sigma, sigma;
  Eigen::Matrix3d inv_sigma = diag_sigma.inverse();
  double term = 1.0 / sqrt(pow(2.0 * M_PI, 3.0) * pow(sigma, 3.0));
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);
  Eigen::Matrix3Xd samples(3, num_samples_);
//  std::vector<GraspHypothesis> hands_new = hands;

  // 3. Find grasp hypotheses using importance sampling.
  for (int i = 0; i < num_iterations_; i++)
  {
    std::cout << i << " " << num_gauss_samples << std::endl;

    // 3.1 Draw samples close to affordances (importance sampling).
    if (this->sampling_method_ == SUM_OF_GAUSSIANS)
    {
      drawSamplesFromSumOfGaussians(hands, generator, sigma, num_gauss_samples, samples);
//      drawSamplesFromSumOfGaussians(hands_new, generator, sigma, num_gauss_samples, samples);
    }
    else if (this->sampling_method_ == MAX_OF_GAUSSIANS) // max of Gaussians
    {
      drawSamplesFromMaxOfGaussians(hands, generator, sigma, num_gauss_samples, samples, term);
//      drawSamplesFromMaxOfGaussians(hands_new, generator, sigma, num_gauss_samples, samples, term);
    }
    else
    {
      drawWeightedSamples(hands, generator, sigma, num_gauss_samples, samples);
    }

    // 3.2 Draw random samples.
    for (int j = num_samples_ - num_rand_samples; j < num_samples_; j++)
    {
      int r = std::rand() % cloud->points.size();
//      while (!pcl::isFinite((*cloud)[r])
//          || !this->affordances.isPointInWorkspace(cloud->points[r].x, cloud->points[r].y, cloud->points[r].z))
//        r = std::rand() % cloud->points.size();
      samples.col(j) = cloud->points[r].getVector3fMap().cast<double>();
    }

    // 3.3 Evaluate grasp hypotheses at <samples>.
    cloud_cam.setSamples(samples);
    std::vector<GraspHypothesis> hands_new = GraspDetector::detectGraspPoses(cloud_cam, false);
//    hands_new = GraspDetector::detectGraspPoses(cloud_cam, false);
    hands.insert(hands.end(), hands_new.begin(), hands_new.end());
    std::cout << "Added/total: " << hands_new.size() << "/" << hands.size() << " grasp hypotheses in round " << i
      << std::endl;

    if (visualize_steps_)
    {
      plotter.plotSamples(samples, cloud);
    }
  }

  std::cout << "Found " << hands.size() << " grasp hypotheses in " << omp_get_wtime() - t0 << " sec.\n";

  if (visualize_results_)
  {
    plotter.plotFingers(hands, cloud_cam.getCloudOriginal(), "All Grasps");

    if (getHandleSearch().getMinInliers() > 0)
    {
      t0 = omp_get_wtime();
      hands = getHandleSearch().findClusters(hands);
      std::cout << "Found " << hands.size() << " clusters in " << omp_get_wtime() - t0 << " sec.\n";
      plotter.plotFingers(hands, cloud_cam.getCloudOriginal(), "Clusters");
    }
  }

  return hands;
}


void ImportanceSampling::drawSamplesFromSumOfGaussians(const std::vector<GraspHypothesis>& hands,
  Gaussian& generator, double sigma, int num_gauss_samples, Eigen::Matrix3Xd& samples_out)
{
  for (std::size_t j = 0; j < num_gauss_samples; j++)
  {
    int idx = rand() % hands.size();
    samples_out(0, j) = hands[idx].getGraspSurface()(0) + generator() * sigma;
    samples_out(1, j) = hands[idx].getGraspSurface()(1) + generator() * sigma;
    samples_out(2, j) = hands[idx].getGraspSurface()(2) + generator() * sigma;
  }
}


void ImportanceSampling::drawSamplesFromMaxOfGaussians(const std::vector<GraspHypothesis>& hands,
  Gaussian& generator, double sigma, int num_gauss_samples, Eigen::Matrix3Xd& samples_out, double term)
{
  int j = 0;
  while (j < num_gauss_samples) // draw samples using rejection sampling
  {
    int idx = rand() % hands.size();
    Eigen::Vector3d x;
    x(0) = hands[idx].getGraspSurface()(0) + generator() * sigma;
    x(1) = hands[idx].getGraspSurface()(1) + generator() * sigma;
    x(2) = hands[idx].getGraspSurface()(2) + generator() * sigma;

    double maxp = 0;
    for (std::size_t k = 0; k < hands.size(); k++)
    {
      double p = (x - hands[k].getGraspSurface()).transpose() * (x - hands[k].getGraspSurface());
      p = term * exp((-1.0 / (2.0 * sigma)) * p);
      if (p > maxp)
        maxp = p;
    }

    double p = (x - hands[idx].getGraspSurface()).transpose() * (x - hands[idx].getGraspSurface());
    p = term * exp((-1.0 / (2.0 * sigma)) * p);
    if (p >= maxp)
    {
      samples_out.col(j) = x;
      j++;
    }
  }
}


void ImportanceSampling::drawWeightedSamples(const std::vector<GraspHypothesis>& hands, Gaussian& generator,
  double sigma, int num_gauss_samples, Eigen::Matrix3Xd& samples_out)
{
  Eigen::VectorXd scores(hands.size());
  double sum = 0.0;
  for (int i = 0; i < hands.size(); i++)
  {
    scores(i) = hands[i].getScore();
    sum += scores(i);
  }

  for (int i = 0; i < hands.size(); i++)
  {
    scores(i) /= sum;
  }

  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::uniform_real<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::uniform_real<> > uniform_generator(*rng, distribution);
//  std::cout << "scores\n" << scores << std::endl;

  for (int i = 0; i < num_gauss_samples; i++)
  {
    double r = uniform_generator();
    double x = 0.0;
    int idx = -1;

    for (int j = 0; j < scores.size(); j++)
    {
      x += scores(j);
      if (r < x)
      {
        idx = j;
        break;
      }
    }

    if (idx > -1)
    {
//      std::cout << "r: " << r << ", idx: " << idx << std::endl;
      samples_out(0,i) = hands[idx].getGraspSurface()(0) + generator() * sigma;
      samples_out(1,i) = hands[idx].getGraspSurface()(1) + generator() * sigma;
      samples_out(2,i) = hands[idx].getGraspSurface()(2) + generator() * sigma;
    }
    else
      std::cout << "Error: idx is " << idx << std::endl;
  }
}
