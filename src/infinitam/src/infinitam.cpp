/* A ROS node which utilizes InfiniTAM
 * Assembled: Northeastern University, 2015
 * 
 * A lot of this code traces back to InfiniTAM:
 * - http://www.robots.ox.ac.uk/~victor/infinitam/index.html
*/

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS Includes
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// System Includes
#include <string>

// InfiniTAM Includes
#include "Engine/OpenNIEngine.h"

// Self Includes
#include <infinitam/volume.h>

using namespace std;
using namespace InfiniTAM::Engine;

/* --- Global Variables --- */

int publishVolume = 0; // 0->void,1->points,2->pointsAndNormals
bool resetTrack = true;

#define NBLOCKS (SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE)
#define NVOXELS (SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3)
ITMHashEntry* hashTable;
ITMVoxel* voxelData;

/* --- Utility Functions --- */

// Adapted from InfiniTAM, ITMRepresentationAccess.h.
Vector3i PointToSDFBlock(Vector3i voxelPos)
{
	if (voxelPos.x < 0) voxelPos.x -= SDF_BLOCK_SIZE - 1;
	if (voxelPos.y < 0) voxelPos.y -= SDF_BLOCK_SIZE - 1;
	if (voxelPos.z < 0) voxelPos.z -= SDF_BLOCK_SIZE - 1;
	return voxelPos / SDF_BLOCK_SIZE;
}

// Adapted from InfiniTAM, ITMRepresentationAccess.h.
template<typename T> int HashIndex(const ITMLib::Vector3<T> voxelPos, const int hashMask)
{
	return ((uint)(((uint)voxelPos.x * 73856093) ^ ((uint)voxelPos.y * 19349669) ^ ((uint)voxelPos.z * 83492791)) & (uint)hashMask);
}

// Adapted from InfiniTAM, ITMRepresentationAccess.h.
int PointPosParse(Vector3i voxelPos, Vector3i& blockPos)
{
	blockPos = PointToSDFBlock(voxelPos);
	Vector3i locPos = voxelPos - blockPos * SDF_BLOCK_SIZE;
	return locPos.x + locPos.y * SDF_BLOCK_SIZE + locPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
}

// Adapted from InfiniTAM, ITMRepresentationAccess.h.
ITMVoxel ReadVoxel(Vector3i point, bool& isFound)
{
	Vector3i blockPos;
  int offsetExcess = 0;

	int linearIdx = PointPosParse(point, blockPos);
	int hashIdx = HashIndex(blockPos, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;

	isFound = false;

	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET; inBucketIdx++) 
	{
		const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess];

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}

		offsetExcess = hashEntry.offset - 1;
	}

	return ITMVoxel();
}

// Adapted from InfiniTAM, ITMSceneReconstructionEngine.h.
Vector3f ComputeNormalFromSDF(Vector3i pos)
{
	bool isFound;
  Vector3f ret;

	// all 8 values are going to be reused several times
	Vector4f front, back;
	front.x = ReadVoxel(pos + Vector3i(0, 0, 0), isFound).sdf;
	front.y = ReadVoxel(pos + Vector3i(1, 0, 0), isFound).sdf;
	front.z = ReadVoxel(pos + Vector3i(0, 1, 0), isFound).sdf;
	front.w = ReadVoxel(pos + Vector3i(1, 1, 0), isFound).sdf;
	back.x  = ReadVoxel(pos + Vector3i(0, 0, 1), isFound).sdf;
	back.y  = ReadVoxel(pos + Vector3i(1, 0, 1), isFound).sdf;
	back.z  = ReadVoxel(pos + Vector3i(0, 1, 1), isFound).sdf;
	back.w  = ReadVoxel(pos + Vector3i(1, 1, 1), isFound).sdf;

	Vector4f tmp;
	float p1, p2, v1;
	// gradient x
	p1 = front.x + front.z + back.x + back.z;
	tmp.x = ReadVoxel(pos + Vector3i(-1, 0, 0), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(-1, 1, 0), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(-1, 0, 1), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(-1, 1, 1), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w;
	v1 = p1 + p2;

	p1 = front.y + front.w + back.y + back.w;
	tmp.x = ReadVoxel(pos + Vector3i(2, 0, 0), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(2, 1, 0), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(2, 0, 1), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(2, 1, 1), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w;

	ret.x = ITMVoxel::SDF_valueToFloat(p1 + p2 - v1);

	// gradient y
	p1 = front.x + front.y + back.x + back.y;
	tmp.x = ReadVoxel(pos + Vector3i(0, -1, 0), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(1, -1, 0), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(0, -1, 1), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(1, -1, 1), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w;
	v1 = p1 + p2;

	p1 = front.z + front.w + back.z + back.w;
	tmp.x = ReadVoxel(pos + Vector3i(0, 2, 0), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(1, 2, 0), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(0, 2, 1), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(1, 2, 1), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w;

	ret.y = ITMVoxel::SDF_valueToFloat(p1 + p2 - v1);

	// gradient z
	p1 = front.x + front.y + front.z + front.w;
	tmp.x = ReadVoxel(pos + Vector3i(0, 0, -1), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(1, 0, -1), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(0, 1, -1), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(1, 1, -1), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w;
	v1 = p1 + p2;

	p1 = back.x + back.y + back.z + back.w;
	tmp.x = ReadVoxel(pos + Vector3i(0, 0, 2), isFound).sdf;
	tmp.y = ReadVoxel(pos + Vector3i(1, 0, 2), isFound).sdf;
	tmp.z = ReadVoxel(pos + Vector3i(0, 1, 2), isFound).sdf;
	tmp.w = ReadVoxel(pos + Vector3i(1, 1, 2), isFound).sdf;
	p2 = tmp.x + tmp.y + tmp.z + tmp.w ;

	ret.z = ITMVoxel::SDF_valueToFloat(p1 + p2 - v1);

	return ret;
}

void PopulatePointsAndNormals(ITMScene<ITMVoxel, ITMVoxelIndex>* scene, double voxelSize, double mu,
  double minX, double maxX, double minY, double maxY, double minZ, double maxZ,
  vector<float>& points, vector<float>& normals)
{
  ITMSafeCall(cudaMemcpy(
    hashTable, scene->index.GetEntries(), NBLOCKS*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
  ITMSafeCall(cudaMemcpy(
    voxelData, scene->localVBA.GetVoxelBlocks(), NVOXELS*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  
  minX /= (SDF_BLOCK_SIZE*voxelSize); maxX /= (SDF_BLOCK_SIZE*voxelSize);
  minY /= (SDF_BLOCK_SIZE*voxelSize); maxY /= (SDF_BLOCK_SIZE*voxelSize);
  minZ /= (SDF_BLOCK_SIZE*voxelSize); maxZ /= (SDF_BLOCK_SIZE*voxelSize);
  
  for (int blockIdx = 0; blockIdx < NBLOCKS; blockIdx++)
  {
    const ITMHashEntry hashEntry = hashTable[blockIdx];
    if (hashEntry.ptr < 0) continue; // not allocated
    
    if (hashEntry.pos.x < minX || hashEntry.pos.x > maxX ||
        hashEntry.pos.y < minY || hashEntry.pos.y > maxY ||
        hashEntry.pos.z < minZ || hashEntry.pos.z > maxZ)
        continue; // outside of user-specified workspace
    
    int p = hashEntry.ptr * SDF_BLOCK_SIZE3;
    
    for (int locIdxZ = 0; locIdxZ < SDF_BLOCK_SIZE; locIdxZ++)
      for (int locIdxY = 0; locIdxY < SDF_BLOCK_SIZE; locIdxY++)
        for (int locIdxX = 0; locIdxX < SDF_BLOCK_SIZE; locIdxX++)
        {
          ITMVoxel v = voxelData[
            p + locIdxX + SDF_BLOCK_SIZE * locIdxY + SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * locIdxZ];
          
          if (v.w_depth == 0) continue;
          
          // How to convert SDF to meters? See ITMSceneReconstructionEngine.h/computeUpdatedVoxelDepthInfo.
          if (mu*(fabs(v.sdf)/32767.0) > voxelSize) continue;
          
          points.push_back(voxelSize*(SDF_BLOCK_SIZE*hashEntry.pos.x + locIdxX));
          points.push_back(voxelSize*(SDF_BLOCK_SIZE*hashEntry.pos.y + locIdxY));
          points.push_back(voxelSize*(SDF_BLOCK_SIZE*hashEntry.pos.z + locIdxZ));
          
          // if (publishVolume == 2)
          // {
            Vector3f normal = ComputeNormalFromSDF(Vector3i(
              SDF_BLOCK_SIZE*hashEntry.pos.x + locIdxX,
              SDF_BLOCK_SIZE*hashEntry.pos.y + locIdxY,
              SDF_BLOCK_SIZE*hashEntry.pos.z + locIdxZ));
            double norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
            normal.x /= norm;
            normal.y /= norm;
            normal.z /= norm;
            
            normals.push_back(normal.x);
            normals.push_back(normal.y);
            normals.push_back(normal.z);
          // }
        }
  }
}

void WaitForPublishVolumeSignal()
{
  ROS_INFO("Waiting for volume signal.");
  
  while (!publishVolume && ros::ok())
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
}

/* --- Receive a Signal --- */

/** Listens to /infinitam/publishVolume for requests to publish the volume.
 * 
 * A volume publish task is expensive so is only done on demand. The client must call this function
 * to receive the desired data.
 * 
 * - Input inputSignal: If 0, resets track, if 1, publishes just points. If 2, publishes points and normals.
 * - Returns void.
 */
void PublishVolumeListener(const std_msgs::Int32::ConstPtr& inputSignal)
{
  if (inputSignal->data == 0)
  {
    resetTrack = true;
    return;
  }
  
  publishVolume = inputSignal->data;
}

/* --- Draw a point cloud with normals --- */
void drawPointsAndNormals(const infinitam::volume& volumeMsg)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(""));  
  // viewer->setBackgroundColor(1, 1, 1);
  viewer->setBackgroundColor(0, 0, 0);
  
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);        
  cloud->width    = volumeMsg.nPoints;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = volumeMsg.points[i*3];
    cloud->points[i].y = volumeMsg.points[i*3+1];
    cloud->points[i].z = volumeMsg.points[i*3+2];
    cloud->points[i].normal_x = volumeMsg.normals[i*3];
    cloud->points[i].normal_y = volumeMsg.normals[i*3+1];
    cloud->points[i].normal_z = volumeMsg.normals[i*3+2];
  }
  
  viewer->addPointCloud<pcl::PointNormal>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.01, "normals");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1,
		"normals");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "normals");
  
  viewer->initCameraParameters();
  viewer->setPosition(0, 0);
	viewer->setSize(640, 480);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
  
  viewer->close();
}

/* --- Main --- */

int main(int argc, char** argv)
{
  // Initialize ROS
  
  ros::init(argc, argv, "infinitam");
  ros::NodeHandle nodeHandle("~");
  
  string calibFile; nodeHandle.param<string>("calibFile", calibFile, "calib.txt");
  
  double mu; nodeHandle.param<double>("mu", mu, 0.02);
  double maxW; nodeHandle.param<double>("maxW", maxW, 100);
  double voxelSize; nodeHandle.param<double>("voxelSize", voxelSize, 0.002);
  double viewFrustumMin; nodeHandle.param<double>("viewFrustumMin", viewFrustumMin, 0.20);
  double viewFrustumMax; nodeHandle.param<double>("viewFrustumMax", viewFrustumMax, 3.00);
  double noHierarchyLevels; nodeHandle.param<double>("noHierarchyLevels", noHierarchyLevels, 5);
  double noRotationOnlyLevels; nodeHandle.param<double>("noRotationOnlyLevels", noRotationOnlyLevels, 3);
  double depthTrackerICPThreshold; nodeHandle.param<double>("depthTrackerICPThreshold", depthTrackerICPThreshold, 0.01);
  double noICPRunTillLevel; nodeHandle.param<double>("noICPRunTillLevel", noICPRunTillLevel, 1);
  double minX; nodeHandle.param<double>("minX", minX, -100);
  double maxX; nodeHandle.param<double>("maxX", maxX,  100);
  double minY; nodeHandle.param<double>("minY", minY, -100);
  double maxY; nodeHandle.param<double>("maxY", maxY,  100);
  double minZ; nodeHandle.param<double>("minZ", minZ, -100);
  double maxZ; nodeHandle.param<double>("maxZ", maxZ,  100);
  double minFrames; nodeHandle.param<double>("minFrames", minFrames, 5);
  bool savesClouds; nodeHandle.param<bool>("savesClouds", savesClouds, false);
  std::string cloudFolder; nodeHandle.param<std::string>("cloudFolder", cloudFolder, "");
  
  ros::Publisher volumePublisher = nodeHandle.advertise<infinitam::volume>("/infinitam/volume", 1);
  ros::Subscriber volumeRequestListener = nodeHandle.subscribe("/infinitam/publishVolume", 1, PublishVolumeListener);
  
  // Initialize InfiniTAM
  
  ITMLibSettings* infinitamSettings = new ITMLibSettings();
  
  infinitamSettings->sceneParams.mu = mu;
  infinitamSettings->sceneParams.voxelSize = voxelSize;
  infinitamSettings->sceneParams.viewFrustum_min = viewFrustumMin;
  infinitamSettings->sceneParams.viewFrustum_max = viewFrustumMax;
  infinitamSettings->noHierarchyLevels = noHierarchyLevels;
  infinitamSettings->noRotationOnlyLevels = noRotationOnlyLevels;
  infinitamSettings->depthTrackerICPThreshold = depthTrackerICPThreshold;
  infinitamSettings->noICPRunTillLevel = noICPRunTillLevel;
  infinitamSettings->useGPU = true;
  
  infinitamSettings->trackerType = (noICPRunTillLevel == 0) ?
    ITMLibSettings::TRACKER_ICP : ITMLibSettings::TRACKER_REN;
  
  ImageSourceEngine* imageSource = NULL;
  ITMIntrinsics depthIntrinsics;
  ITMMainEngine* mainEngine = NULL;
  
  hashTable = new ITMHashEntry[NBLOCKS];
  voxelData = new ITMVoxel[NVOXELS];
  
  // Main Loop
  
  int frameCounter = 0;
  int trackCounter = 0;
  int publishCounter = 0;
  ros::Time lastScreenPrint = ros::Time::now();
  
  int lastDataSize = 0;
  
  while (ros::ok())
  { 
    // Check for Reset
    
    if (resetTrack)
    {
      delete mainEngine; mainEngine = NULL;
      delete imageSource; imageSource = NULL;
      
      ROS_WARN("Reset tracker. Track count is at %d.\n", ++trackCounter);
      
      WaitForPublishVolumeSignal();
      if (!ros::ok()) break;
      
      resetTrack = false;
      publishCounter = 0;
      
      imageSource = new OpenNIEngine(calibFile.c_str(), NULL /*device_uri*/);
      ITMIntrinsics depthIntrinsics = imageSource->calib.intrinsics_d;
      mainEngine = new ITMMainEngine(infinitamSettings, &imageSource->calib,
        imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
      
      ROS_INFO("Loaded Calibration Params: fx=%f, fy=%f, px=%f, py=%f.",
        depthIntrinsics.projectionParamsSimple.fx, depthIntrinsics.projectionParamsSimple.fy,
        depthIntrinsics.projectionParamsSimple.px, depthIntrinsics.projectionParamsSimple.py);
    }
    
    // Process Image
    
    imageSource->getImages(mainEngine->view);
    mainEngine->ProcessFrame();
    
    // Retrieve Data
    
    Matrix4f T = mainEngine->trackingState->pose_d->M;
    
    if (++publishCounter > minFrames && publishVolume)
    {
      infinitam::volume volumeMsg;
      volumeMsg.voxelSize = voxelSize;
      
      volumeMsg.points = std::vector<float>();
      volumeMsg.normals = std::vector<float>();
      volumeMsg.points.reserve(lastDataSize);
      if (publishVolume == 2) volumeMsg.normals.reserve(lastDataSize);
      
      PopulatePointsAndNormals(mainEngine->scene, voxelSize, mu, minX, maxX, minY, maxY, minZ, maxZ,
        volumeMsg.points, volumeMsg.normals);
      volumeMsg.nPoints = (int)volumeMsg.points.size() / 3;
      lastDataSize = (int)volumeMsg.points.size();
      
      volumeMsg.pose = std::vector<float>(16);
      std::cout << "Pose (colum by column): ";
      for (int i = 0; i < 16; i++)
      {
        volumeMsg.pose[i] = T.m[i];
        std::cout << volumeMsg.pose[i]  << ", ";
      }
      std::cout << std::endl;
      
      volumeMsg.trackCount = trackCounter;
      
      ROS_INFO("Publishing cloud with %d points", volumeMsg.nPoints);      
      volumePublisher.publish(volumeMsg);
      publishVolume = 0;
      
      //drawPointsAndNormals(volumeMsg);
      
      // save point cloud in a file
      if (savesClouds)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);        
        cloud->width    = volumeMsg.nPoints;
        cloud->height   = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
          cloud->points[i].x = volumeMsg.points[i*3];
          cloud->points[i].y = volumeMsg.points[i*3+1];
          cloud->points[i].z = volumeMsg.points[i*3+2];
        }
        
        std::string filename = "infinitam_" + boost::lexical_cast<std::string>(ros::Time::now().toSec()) + ".pcd";
        filename = cloudFolder + filename;
        pcl::io::savePCDFileASCII (filename, *cloud);
        ROS_INFO_STREAM("Saved " << cloud->points.size () << " data points to " << filename);
      }
    }
    
    // Print Frame Rate
    
    if (++frameCounter % 30 == 0)
    {
      ros::Time now = ros::Time::now();
      ROS_INFO("Processing at %f fps.", 30.0/(now-lastScreenPrint).toSec());
      lastScreenPrint = now;
    }
    
    ros::spinOnce();
  }
  
  delete[] voxelData;
  delete[] hashTable;
  delete mainEngine;
  delete imageSource;
  delete infinitamSettings;
  
  return 0;
}

