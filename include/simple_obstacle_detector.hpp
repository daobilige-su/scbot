#ifndef SIMPLE_OBSTACLE_DETECTOR_HPP_
#define SIMPLE_OBSTACLE_DETECTOR_HPP_

#include "pose_transformer.hpp"
#include "scbot_objects.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <memory>
#include <mutex>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace scbot { // namespace for solar panel clearer robot

class SimpleObstacleDetector {

public:
  // constructors
  SimpleObstacleDetector();

  // setters
  bool
  setViewer(std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_ptr);
  bool setViewerMutex(std::shared_ptr<std::mutex> viewer_mutex_ptr);
  bool setTransform(std::vector<float> pose3d_trans_ypr);
  bool setPc(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);
  bool setDebugVerbose(bool flag);
  bool setUseRobotCoordInBbox(bool flag);

  // getters
  const std::vector<ObstacleObj> getObstacleObjs();
  const pcl::PointCloud<pcl::PointXYZ>::Ptr getPcTransform() const;

private:
  std::vector<ObstacleObj> obstacle_objs_;
  Eigen::Matrix4f transform_lidar_robot_ = Eigen::Matrix4f::Identity();
  std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_ = nullptr;
  std::shared_ptr<std::mutex> pcl_viewer_mutex_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_raw_ptr_ = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_robot_transformed_ptr_ = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_roi_ptr_ = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sp_ptr_ = nullptr;

  bool debug_verbose_ = true;
  bool use_robot_coord_in_bbox_ = false;
};
} // namespace scbot

#endif