#include <ros/ros.h>

#include <nodes/grasp_detection_node.h>


int main(int argc, char** argv)
{
  // initialize ROS
  ros::init(argc, argv, "detect_grasps");
  ros::NodeHandle node("~");
  
  // read filename from parameter
  std::string cloud_file_name;
  std::string normals_file_name;
  node.getParam("cloud_file_name", cloud_file_name);
  node.param<std::string>("normals_file_name", normals_file_name, std::string(""));
  std::string file_name_left;
  std::string file_name_right;
  bool use_normals_file;

  if (cloud_file_name.find(".pcd") == std::string::npos)
  {
    file_name_left = cloud_file_name + "l_reg.pcd";
    file_name_right = cloud_file_name + "r_reg.pcd";
    use_normals_file = false;
  }
  else if (normals_file_name.size() > 0)
  {
    file_name_left = cloud_file_name;
    file_name_right = normals_file_name;
    use_normals_file = true;
    std::cout << "Using normals file: " << normals_file_name << "\n";
  }
  else
  {
    file_name_left = cloud_file_name;
    file_name_right = "";
    use_normals_file = false;
  }

  GraspDetectionNode grasp_detection(node);
  grasp_detection.detectGraspPosesInFile(file_name_left, file_name_right, use_normals_file);
  
  return 0;
}
