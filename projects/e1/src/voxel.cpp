#include "voxel.h"

using namespace std;


void renderEnvironment(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int side, 
  float cameraDistance, float cameraRotation, int& id)
{
  viewer.addCoordinateSystem(1.0);

  // Define unique rendering ids
  string pointCloud = "cloud" + id;
  string cage = "cage" + id;
  string cube = "cube" + id;
  string cubeFill = "cubeFill" + id;

  // Add point cloud
  viewer.addPointCloud(cloud, pointCloud);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, pointCloud);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, .726, .502, 0., pointCloud);

  // Add environment wireframe
  viewer.addCube(0, side, 0, side, 0, side, 1, 1, 1, cage);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cage);

  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, cage);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.5, cage);

  // Add target voxel wireframe
  viewer.addCube(0, side/2, 0, side/2, 0, side/2, 255, 0, 255, cube);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);

  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 1., cube);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.5, cube);

  // Add target voxel surface
  viewer.addCube(0, side/2, 0, side/2, 0, side/2, 255, 0, 255, cubeFill);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 1., cubeFill);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, cubeFill);

  viewer.setCameraPosition(cameraRotation, cameraRotation, cameraRotation, 0, 0, 1);

  while(!viewer.wasStopped())
    viewer.spinOnce();

  id++;
}

int main(int argc, char const *argv[])
{ 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Define enclosing environment side
  int side = 4;

  // Randomly populate environment
  for (int i=0; i < 1000; i++)
  {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dist(0, side);

    double x = dist(gen);
    double y = dist(gen);
    double z = dist(gen);

    pcl::PointXYZ point = pcl::PointXYZ(x, y, z);

    cloud->push_back(point);
  }

  int id = 0;

  // Zoom out and rotate camera view by 45° along roll (X), pitch (Y), yaw (Z)
  float cameraDistance = 14;
  float cameraRotation = cameraDistance * M_PI / 4;

  pcl::visualization::PCLVisualizer viewer("Input Point Cloud");
  renderEnvironment(viewer, cloud, side, cameraDistance, cameraRotation, id);

  // Filter input cloud using voxels
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
  
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud);

  // The filtered cloud will have 2^3 = 8 points
  voxelGrid.setLeafSize(side/2, side/2, side/2);
  voxelGrid.filter(*cloudFiltered);

  pcl::visualization::PCLVisualizer voxelViewer("Voxel Cloud");
  renderEnvironment(voxelViewer, cloudFiltered, side, cameraDistance, cameraRotation, id);

  return 0;
}