#include <ros/ros.h>

#include <agile_grasp2/grasp_detector.h>
#include <agile_grasp2/hand_search.h>
#include <agile_grasp2/learning.h>
#include <agile_grasp2/plot.h>


int main(int argc, char** argv)
{
  // initialize ROS
  ros::init(argc, argv, "generate_training_data");
  ros::NodeHandle node("~");

  double nn_radius_hands_ = 0.1;
  int num_cams = 20;

  // read filename from parameter
  std::string cloud_folder;
  node.getParam("cloud_folder", cloud_folder);

  double hand_height;
  node.param<double>("hand_height", hand_height);

  boost::filesystem::path path(cloud_folder);
  boost::filesystem::directory_iterator it(path);
  GraspDetector grasp_detector(node);

  // sort files alphabetically
  int num_objects = 0;
  std::vector<boost::filesystem::path> files;
  while (it != boost::filesystem::directory_iterator())
  {
    if ((*it).path().string().find("gt") == std::string::npos)
    {
      files.push_back((*it).path());
    }
    else
    {
      num_objects++;
    }
    it++;
  }
  std::sort(files.begin(), files.end());
  std::cout << "Going to process " << num_objects << " objects.\n";

  Plot plot;
  bool is_plotting = false;
  std::vector<GraspHypothesis> positives, negatives;
  positives.reserve(num_objects*num_cams*20);
  negatives.reserve(num_objects*num_cams*20);

//  for (int i = 0; i < files.size(); i++)
//  for (int i = 0; i < num_objects; i++)
  for (int i = 0; i < 10; i++)
  {
    std::string filepath = files[i*num_cams].string();
    int start = filepath.find_last_of("/") + 1;
    std::string object = filepath.substr(start, filepath.find_last_of("_") - start);
    std::cout << "====== Current Object: " << object << '\n';

    // load mesh for ground truth
    std::string mesh_file_path = cloud_folder + "/" + object + "_gt.pcd";
    std::cout << " mesh_file_path: " << mesh_file_path << '\n';
    CloudCamera mesh_cloud_cam(mesh_file_path);

    // load normals for ground truth
    std::string normals_file_path = cloud_folder + "/" + object + "_gt_normals.csv";
    std::cout << " normals_file_path: " << normals_file_path << '\n';
    mesh_cloud_cam.setNormalsFromFile(normals_file_path);
//      mesh_cloud_cam.voxelizeCloud(0.001);
    std::cout << "mesh after voxelization: " << mesh_cloud_cam.getCloudProcessed()->size() << " points.\n";
    int num_pos_obj = 0;
    int num_neg_obj = 0;

//    for (int j = 0; j < num_cams; j++)
    for (int j = 0; j < 10; j++)
    {
      filepath = files[i*num_cams + j].string();

      if (filepath.find("gt") == std::string::npos)
      {
        std::cout << "Processing " << filepath << '\n';

        // 1. Find grasps in point cloud.
        CloudCamera view_cloud_cam(filepath);
        grasp_detector.preprocessPointCloud(view_cloud_cam);
        std::vector<GraspHypothesis> grasps = grasp_detector.generateHypotheses(view_cloud_cam);

        // 2. Evaluate grasps using mesh as ground truth.
        double t0 = omp_get_wtime();
        ROS_INFO("Evaluating grasp hypotheses on ground truth ...");
        grasp_detector.evaluateGraspPoses(mesh_cloud_cam, grasps, nn_radius_hands_);
        ROS_INFO_STREAM(" runtime: " << omp_get_wtime() - t0);

        std::vector<GraspHypothesis> pos;
        std::vector<GraspHypothesis> neg;
        for (int k = 0; k < grasps.size(); k++)
        {
  //        std::cout << j << " -- half-antipodal: " << grasps[j].isHalfAntipodal() << ", full-antipodal: "
  //          << grasps[j].isFullAntipodal() << std::endl;
          if (grasps[k].isFullAntipodal())
            pos.push_back(grasps[k]);
          else
            neg.push_back(grasps[k]);
        }
        std::cout << "Found " << pos.size() << " positives and " << neg.size() << " negatives for view " << j << "\n";

        // 3. Equalize the number of positives and negatives.
        if (pos.size() <= neg.size())
        {
          positives.insert(positives.end(), pos.begin(), pos.end());
          negatives.insert(negatives.end(), neg.begin(), neg.begin() + pos.size());
          num_pos_obj += pos.size();
          num_neg_obj += pos.size();
        }
        else
        {
          negatives.insert(negatives.end(), neg.begin(), neg.end());
          positives.insert(positives.end(), pos.begin(), pos.begin() + neg.size());
          num_pos_obj += neg.size();
          num_neg_obj += neg.size();
        }
        std::cout << "Now have " << num_pos_obj << " positives and " << num_neg_obj << " negatives.\n";
        std::cout << "--------------------------\n\n";

        if (is_plotting)
        {
          plot.plotFingers(grasps, view_cloud_cam.getCloudProcessed(), "Grasp Hypotheses (view cloud)");
  //        plot.plotFingers(grasps, mesh_cloud_cam.getCloudProcessed(), "Grasp Hypotheses (mesh)");

          plot.plotFingers(pos, view_cloud_cam.getCloudProcessed(), "Antipodal Grasps (view cloud)");
  //        plot.plotFingers(pos, mesh_cloud_cam.getCloudProcessed(), "Antipodal Grasps (mesh)");
        }
      }
      else
      {
        ROS_ERROR("Something went wrong!");
      }
    }

    // equalize the number of positives and negatives
    int obj_name_start = filepath.find_last_of("/");
    int obj_name_end = filepath.find_last_of("_");
    std::string obj_name = filepath.substr(obj_name_start + 1, obj_name_end - obj_name_start - 1);
    std::cout << "Found " << num_pos_obj << " positives and " << num_neg_obj << " negatives for " << obj_name  << "\n";
    std::cout << "Total: " << positives.size() << " positives and " << negatives.size() << " negatives \n";
    std::cout << "==========================\n\n";
  }

  // store the training data
  Eigen::Matrix3Xd fake_cam_pos;
  std::vector<GraspHypothesis> training_data;
  training_data.insert(training_data.end(), positives.begin(), positives.end());
  training_data.insert(training_data.end(), negatives.begin(), negatives.end());
  Learning learning(60, 1, hand_height);
  learning.storeLabeledGraspImages(training_data, fake_cam_pos, cloud_folder + "/training_data/");

  return 0;
}
