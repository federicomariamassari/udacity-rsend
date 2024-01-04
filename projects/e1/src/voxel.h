#include <iostream>
#include <random>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>


/**
 * @brief Render PCL Viewer environment (requires disabling GPU acceleration).
 * 
 * @param viewer Input PCL Viewer object.
 * @param cloud Input point cloud.
 * @param side Input side of the environment wireframe.
 * @param cameraDistance Camera distance from the rendered environment.
 * @param cameraRotation Camera rotation parameter, applied to all cube sides.
 * @param id Unique rendering id.
 */
void renderEnvironment(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int side, 
  float cameraDistance, float cameraRotation, int& id);
