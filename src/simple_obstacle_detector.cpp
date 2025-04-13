#include "simple_obstacle_detector.hpp"
#include "scbot_objects.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_config.h>
#include <sstream>
#include <string>

namespace scbot
{
// utility functions declarations
pcl::PointCloud<pcl::PointXYZ>::Ptr
ObsDetExtractPcByXYZLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                           float x_min, float x_max, float y_min, float y_max,
                           float z_min, float z_max);
pcl::PointCloud<pcl::PointXYZ>::Ptr CropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                           float x_min, float x_max, float y_min, float y_max,
                           float z_min, float z_max);
void PrintObstacleParam(const std::vector<ObstacleObj> &obstacle_objs);

double time_now();

struct pc_bbox_res {
  Eigen::Quaternionf bboxQuaternion;
  Eigen::Vector3f bboxTransform;
  pcl::PointXYZ minPoint;
  pcl::PointXYZ maxPoint;
};
pc_bbox_res compute_pc_2dbbox(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr);

// class member function definitions
SimpleObstacleDetector::SimpleObstacleDetector() {
  // init pc (pcl pointcloud must be initialized!!)
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pc_transformed_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pc_roi_ptr_r(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pc_sp_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  
  pc_raw_ptr_ = empty_pc_ptr;
  pc_robot_transformed_ptr_ = empty_pc_transformed_ptr;
  pc_roi_ptr_ = empty_pc_roi_ptr_r;
  pc_sp_ptr_ = empty_pc_sp_ptr;
}

const std::vector<ObstacleObj> SimpleObstacleDetector::getObstacleObjs() {
  // params
  float pc_x_limit_min = -10.0;
  float pc_x_limit_max = 10.0;
  float pc_y_limit_min = -10.0;
  float pc_y_limit_max = 10.0;
  float pc_z_limit_min = -3.0;
  float pc_z_limit_max = 3.0;

  float pc_robot_frame_x_min = -0.5;
  float pc_robot_frame_x_max = 0.5;
  float pc_robot_frame_y_min = -0.0;
  float pc_robot_frame_y_max = 2.0;
  float pc_robot_frame_z_min = -3.0;
  float pc_robot_frame_z_max = 3.0;

  float plane_seg_dist_thr = 0.05;

  float ang_to_Z_ref = 40.0;

  Eigen::Vector3f sp_plane_norm_ref(0, -0.707, 0.707);
  float sp_plane_norm_diff_thr = 0.2;

  float sp_det_range = 10.0;
  float sub_pc_x_size = 0.2;
  int sub_pc_min_pts = 50;

  float sub_pc_top_z_ref = -0.55;
  float sub_pc_top_z_ref_thr = 0.5;
  int top_line_can_num = 5;

  float panel_width = 4.0;

  float gap_start_shift = 0.2;
  float gap_det_range = 5.0;
  float gap_det_res = 0.02;

  poseTransformer::poseTransformer transformer;

  // reset obstacle_objs_
  obstacle_objs_.clear();
  // return if raw pc is not set.
  if (pc_raw_ptr_ == nullptr)
  {
    std::cout << "Warning: in simple_obstacle_detector: Raw Point Could Ptr "
                 "is NullPtr, skipping ..."
              << std::endl;
    return obstacle_objs_;
  }

  // (1) transform pc to robot frame
  if (debug_verbose_)
  {
    std::cout << "transform_bottom_lidar_robot_: " << std::endl;
    std::cout << transform_lidar_robot_ << std::endl;
  }
  pcl::transformPointCloud(*pc_raw_ptr_, *pc_robot_transformed_ptr_,
                           transform_lidar_robot_);

  // (2) Extract pc close to robot, and exclude frames of the robot
  pc_roi_ptr_ = ObsDetExtractPcByXYZLimits(
      pc_robot_transformed_ptr_, pc_x_limit_min, pc_x_limit_max, pc_y_limit_min,
      pc_y_limit_max, pc_z_limit_min, pc_z_limit_max);

  // remove robot boday
  pc_roi_ptr_ =
      CropBoxFilter(pc_roi_ptr_, pc_robot_frame_x_min, pc_robot_frame_x_max,
                    pc_robot_frame_y_min, pc_robot_frame_y_max,
                    pc_robot_frame_z_min, pc_robot_frame_z_max);

  // pcl::CropBox<pcl::PointXYZ> boxFilter;
  // boxFilter.setMin(Eigen::Vector4f(pc_robot_frame_x_min,
  // pc_robot_frame_y_min,
  //                                  pc_robot_frame_z_min, 1.0));
  // boxFilter.setMax(Eigen::Vector4f(pc_robot_frame_x_max,
  // pc_robot_frame_y_max,
  //                                  pc_robot_frame_z_max, 1.0));
  // boxFilter.setMin(filtermin);
  // boxFilter.setMax(filtermax);
  // boxFilter.setInputCloud(tmp_pc_ptr_);
  // std::cout << PCL_VERSION << std::endl;
  // std::cout<< boxFilter.getMin() << std::endl << boxFilter.getMax() <<
  // std::endl; boxFilter.filter(*pc_roi_ptr_); pcl::PassThrough<pcl::PointXYZ>
  // pass; pass.setInputCloud(pc_roi_ptr_); pass.setFilterFieldName("x");
  // pass.setFilterLimits(pc_robot_frame_x_min, pc_robot_frame_x_max);
  // pass.setNegative(true);
  // pass.filter(*pc_roi_ptr_);

  // visualization
  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);
    pcl_viewer_->removePointCloud("od_pc_robot_transformed");
    pcl_viewer_->removePointCloud("od_pc_roi");
    if (debug_verbose_) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_robot_transformed_color_handle(pc_robot_transformed_ptr_, 70, 70,
                                            70);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(
          pc_robot_transformed_ptr_, pc_robot_transformed_color_handle,
          "od_pc_robot_transformed");
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        pc_roi_color_handle(pc_roi_ptr_, 255, 255, 255);
    pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_roi_ptr_, pc_roi_color_handle,
                                              "od_pc_roi");
  }

  // (3) extract ground plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_no_ground_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ground_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  float floor_z = -2.3;
  float floor_max_height = 0.2;

  for(const auto& pt : *pc_roi_ptr_){
    if(pt.z>(floor_z+floor_max_height)){
      pc_no_ground_ptr->push_back(pt);
    } else{
      pc_ground_ptr->push_back(pt);
    }
  }
  // visualization
  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);
    pcl_viewer_->removePointCloud("od_pc_no_ground");
    pcl_viewer_->removePointCloud("od_pc_ground");
    if (debug_verbose_) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_no_ground_color_handle(pc_no_ground_ptr, 255, 255,
                                            0);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(
          pc_no_ground_ptr, pc_no_ground_color_handle,
          "od_pc_no_ground");
    
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_ground_color_handle(pc_ground_ptr, 0, 255, 0);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_ground_ptr, pc_ground_color_handle,
                                                "od_pc_ground");
    }
  }



  // (4) extract solar panel plane model
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_remain_ptr(pc_no_ground_ptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sp_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_non_sp_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_sp_vec;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_obs_vec;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  
  seg.setDistanceThreshold(plane_seg_dist_thr);

  bool plane_found = true;
  int plane_found_num = 0;
  while (plane_found) {
    if (pc_remain_ptr->size()==0) {
      std::cout << "SimpleObstacleDector: no remaining pc after planner"
                << std::endl;
      plane_found = false;
      break;
    }

    seg.setInputCloud(pc_remain_ptr);
    seg.segment(*inliers, *coefficients);

    if ((plane_found_num == 0) &&
        (inliers->indices.size() == pc_remain_ptr->size())) {
      std::cout << "SimpleObstacleDector: all points are on a plane, returning."
                << std::endl;
      return obstacle_objs_;
    }
    else {
      plane_found_num++;
    }

    if (debug_verbose_) {
      std::cout << "Exact plane points number = " << inliers->indices.size()
                << std::endl;
    }
    
    if (inliers->indices.size() < 30) {
      plane_found = false;
      break;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract; // Create the filtering object
    extract.setInputCloud(pc_remain_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false); // remove these points
    extract.filter(*pc_tmp_ptr);
    extract.setNegative(true); // remove these points
    extract.filter(*pc_remain_ptr);

    Eigen::Vector3f plane_norm(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);
    plane_norm = plane_norm / plane_norm.norm();
    if (plane_norm[2]<0) {
      plane_norm = plane_norm * (-1.0);
    }

    float ang_to_Z = atan2(sqrt(pow(plane_norm[0], 2) + pow(plane_norm[1], 2)),
                           plane_norm[2]) *
                     (180.0 / M_PI);
    if (debug_verbose_) {
      std::cout << "Plane Model coefficients: " << coefficients->values[0]
                << ", " << coefficients->values[1] << ", "
                << coefficients->values[2] << ", " << coefficients->values[3]
                << std::endl;
      std::cout << "Plane Model inliers: " << inliers->indices.size()
                << std::endl;
      std::cout << "ang_to_Z: " << ang_to_Z << std::endl;
    }

    if (abs(ang_to_Z - ang_to_Z_ref) < 5) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(pc_tmp_ptr);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(1.0); // 20cm
      ec.setMinClusterSize(50);
      ec.setMaxClusterSize(25000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc_tmp_ptr);
      ec.extract(cluster_indices);

      for (const auto &cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices) {
          cloud_cluster->push_back((*pc_tmp_ptr)[idx]);
        } //*

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if(debug_verbose_){
          std::cout << "PointCloud representing the Cluster: "
                    << cloud_cluster->size() << " data points." << std::endl;
        }
        pc_sp_vec.emplace_back(*cloud_cluster);
      }

      // pc_sp_vec.emplace_back(*pc_tmp_ptr);
      if (debug_verbose_) {
        std::cout << "Is planner: True" << std::endl;
      }
    }else {
      *pc_non_sp_ptr += *pc_tmp_ptr;
      if (debug_verbose_) {
        std::cout << "Is planner: False" << std::endl;
      }
    }

    if (debug_verbose_) {
      std::cout << "pc_remain_ptr size: " << pc_remain_ptr->size() << std::endl;
    }

    if (pc_sp_vec.size() >= 5) {
      break;
    }
  }

  *pc_non_sp_ptr += *pc_remain_ptr;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pc_non_sp_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.5); // 20cm
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc_non_sp_ptr);
  ec.extract(cluster_indices);

  int j = 0;
  for (const auto &cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster.indices) {
      cloud_cluster->push_back((*pc_non_sp_ptr)[idx]);
    } //*

    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    if (debug_verbose_) {
      std::cout << "PointCloud representing the Cluster: "
                << cloud_cluster->size() << " data points." << std::endl;
    }
    pc_obs_vec.emplace_back(*cloud_cluster);

    j++;
    if(j>=10){break;}
  }

  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);

    std::stringstream ss;
    for (int i = 0; i < 5; ++i) {
      ss.str(std::string());
      ss << "sp_" << i;
      pcl_viewer_->removePointCloud(ss.str());
      pcl_viewer_->removeShape(ss.str() + "_bbox");
    }

    for (int i = 0; i < pc_sp_vec.size(); ++i) {
      ss.str(std::string());
      ss << "sp_" << i;
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc_sp_ptr = pc_sp_vec[i].makeShared();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_sp_vec_color_handle(tmp_pc_sp_ptr, 255, 0, 0);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(
          tmp_pc_sp_ptr, pc_sp_vec_color_handle, ss.str());

      // pcl_viewer_->setShapeRenderingProperties(
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 0);
      pc_bbox_res bbox = compute_pc_2dbbox(tmp_pc_sp_ptr);
      pcl_viewer_->addCube(
          bbox.bboxTransform, bbox.bboxQuaternion,
          bbox.maxPoint.x - bbox.minPoint.x, bbox.maxPoint.y - bbox.minPoint.y,
          bbox.maxPoint.z - bbox.minPoint.z, ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
          ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
          ss.str() + "_bbox");
      if (debug_verbose_) {
        std::cout << "sp " << i << ":" << std::endl;
        std::cout << "bboxTransform: " << bbox.bboxTransform << std::endl;
        std::cout << "bboxQuaternion: " << bbox.bboxQuaternion.coeffs() << std::endl;
        std::cout << "minPoint: " << bbox.minPoint << std::endl;
        std::cout << "maxPoint: " << bbox.maxPoint << std::endl;
      }
    }

    for (int i = 0; i < 10; ++i) {
      ss.str(std::string());
      ss << "obs_" << i;
      pcl_viewer_->removePointCloud(ss.str());
      pcl_viewer_->removeShape(ss.str() + "_bbox");
    }

    for (int i = 0; i < pc_obs_vec.size(); ++i) {
      ss.str(std::string());
      ss << "obs_" << i;
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc_obs_ptr =
          pc_obs_vec[i].makeShared();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_obs_vec_color_handle(tmp_pc_obs_ptr, 0, 0, 255);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(
          tmp_pc_obs_ptr, pc_obs_vec_color_handle, ss.str());

      pc_bbox_res bbox = compute_pc_2dbbox(tmp_pc_obs_ptr);
      pcl_viewer_->addCube(
          bbox.bboxTransform, bbox.bboxQuaternion,
          bbox.maxPoint.x - bbox.minPoint.x, bbox.maxPoint.y - bbox.minPoint.y,
          bbox.maxPoint.z - bbox.minPoint.z, ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0,
          ss.str() + "_bbox");
      pcl_viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
          ss.str() + "_bbox");

      // fill object
      Eigen::Vector4f quat(
          bbox.bboxQuaternion.coeffs()[1], bbox.bboxQuaternion.coeffs()[2],
          bbox.bboxQuaternion.coeffs()[3], bbox.bboxQuaternion.coeffs()[0]);
      Eigen::Vector3f euler = transformer.quatToEuler(quat);
      float heading = euler[0];

      Eigen::Vector3f ctr = bbox.bboxTransform;
      float bbox_len = bbox.maxPoint.x - bbox.minPoint.x;
      float bbox_wid = bbox.maxPoint.y - bbox.minPoint.y;
      float bbox_hei = bbox.maxPoint.z - bbox.minPoint.z;

      // std::time_t result = std::time(nullptr);
      // std::cout << std::asctime(std::localtime(&result)) << result
      //           << " seconds since the Epoch\n";

      ObstacleObj obs_obj;
      obs_obj.isObjValid = true;
      obs_obj.ID = i;
      obs_obj.type = ObstacleType::OBJ_T_UNKNOWN;
      obs_obj.center_x = ctr[0];
      obs_obj.center_y = ctr[1];
      obs_obj.center_z = ctr[2];
      obs_obj.speed_x = 0.0;
      obs_obj.speed_y = 0.0;
      obs_obj.speed_z = 0.0;
      obs_obj.heading = heading;
      obs_obj.box_length = bbox_len;
      obs_obj.box_width = bbox_wid;
      obs_obj.box_height = bbox_hei;
      obs_obj.confidence = 0.75F;
      obs_obj.time_stamp = time_now();

      obstacle_objs_.emplace_back(obs_obj);
    }

    // pcl_viewer_->addLine<pcl::PointXYZ>(top_line_pt1_pcl, top_line_pt2_pcl,
    // 0,
    //                                       0, 1, "top_line");
    // pcl_viewer_->addSphere(top_line_pt1_pcl, 0.2, 0.5, 0.5, 0.0,
    //                        "top_line_sphere1"); // radius, r,g,b
    // pcl_viewer_->spinOnce();
  }

  PrintObstacleParam(obstacle_objs_);
  return obstacle_objs_;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr
SimpleObstacleDetector::getPcTransform() const {
  return pc_robot_transformed_ptr_;
}

bool SimpleObstacleDetector::setPc(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr) {
  pcl::copyPointCloud(*ptr, *pc_raw_ptr_);  // pcl::PointXYZI to pcl::PointXYZ

  return true;
}

bool SimpleObstacleDetector::setViewer(
    std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_ptr) {
  pcl_viewer_ = viewer_ptr;

  return true;  // if you forget to return true, will get a "illegal instruction"
                // error!!! which lead you to think check share_ptr usage, but
                // that is actually right.
}

bool SimpleObstacleDetector::setViewerMutex(
    std::shared_ptr<std::mutex> viewer_mutex_ptr) {
  pcl_viewer_mutex_ptr_ = viewer_mutex_ptr;

  return true;
}

bool SimpleObstacleDetector::setTransform(std::vector<float> pose3d_trans_ypr) {
  Eigen::VectorXf trans_euler(6);
  trans_euler << pose3d_trans_ypr[0], pose3d_trans_ypr[1], pose3d_trans_ypr[2],
      pose3d_trans_ypr[3], pose3d_trans_ypr[4], pose3d_trans_ypr[5];

  poseTransformer::poseTransformer transformer;
  transform_lidar_robot_ = transformer.pose3dEulerToM(trans_euler);

  return true;
}

bool SimpleObstacleDetector::setDebugVerbose(bool flag) {
  debug_verbose_ = flag;

  return true;
}

// utility functions
pcl::PointCloud<pcl::PointXYZ>::Ptr
ObsDetExtractPcByXYZLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                           float x_min, float x_max, float y_min, float y_max,
                           float z_min, float z_max) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud(pc_in);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*pc_out);

  pass.setInputCloud(pc_out);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*pc_out);

  pass.setInputCloud(pc_out);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*pc_out);

  return pc_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
CropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, float x_min,
              float x_max, float y_min, float y_max, float z_min, float z_max) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (const auto &pt : *pc_in) {
    if (pt.x < x_min || pt.x > x_max || pt.y < y_min || pt.y > y_max ||
        pt.z < z_min || pt.z > z_max) {
      pc_out->push_back(pt);
    }
  }
  
  return pc_out;
}

// bool isObjValid = false;
// int ID = 0;
// ObstacleType type = ObstacleType::OBJ_T_UNKNOWN;
// double center_x = 0.0;
// double center_y = 0.0;
// double center_z = 0.0;
// double speed_x = 0.0;
// double speed_y = 0.0;
// double speed_z = 0.0;
// double heading = 0.0;
// double box_length = 1.0;
// double box_width = 1.0;
// double box_height = 1.0;
// float confidence = 0.75F;
// double time_stamp = 0.0;
void PrintObstacleParam(const std::vector<ObstacleObj> &obstacle_objs) {
  int obj_idx = 0;
  for (const auto &obj : obstacle_objs) {
    ++obj_idx;
    std::cout << "Obstacle Object " << obj_idx << ":" << std::endl;
    std::cout << "{" << std::endl;
    std::cout << "  isObjValid: " << obj.isObjValid << std::endl;
    std::cout << "  ID: " << obj.ID << std::endl;
    std::cout << "  type: " << obj.type << std::endl;
    std::cout << "  center_x: " << obj.center_x << std::endl;
    std::cout << "  center_y: " << obj.center_y << std::endl;
    std::cout << "  center_z: " << obj.center_z << std::endl;
    std::cout << "  speed_x: " << obj.speed_x << std::endl;
    std::cout << "  speed_y: " << obj.speed_y << std::endl;
    std::cout << "  speed_z: " << obj.speed_z << std::endl;
    std::cout << "  heading: " << obj.heading << std::endl;
    std::cout << "  box_length: " << obj.box_length << std::endl;
    std::cout << "  box_width: " << obj.box_width << std::endl;
    std::cout << "  box_height: " << obj.box_height << std::endl;
    std::cout << "  confidence: " << obj.confidence << std::endl;
    std::cout << "  time_stamp: " << obj.time_stamp << std::endl;
    std::cout << "}" << std::endl;
  }
}

pc_bbox_res compute_pc_2dbbox(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr) {
  pcl::PointXYZ pt_max, pt_min;
  pcl::getMinMax3D(*pc_ptr, pt_min, pt_max);
  float Z_min = pt_min.z;
  float Z_max = pt_max.z;

  pcl::PointCloud<pcl::PointXYZ> pc_copy = *pc_ptr;
  for (auto &pt : pc_copy) {
    pt.z = 0;
  }


  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(pc_copy, pcaCentroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(pc_copy, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
  /// This line is necessary for proper orientation in some cases. The
           /// numbers come out the same without it, but
           ///    the signs are different and the box doesn't get correctly
           ///    oriented in some cases.
  /* // Note that getting the eigenvectors can also be obtained via the PCL PCA
  interface with something like: pcl::PointCloud<pcl::PointXYZ>::Ptr
  cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloudSegmented);
  pca.project(*cloudSegmented, *cloudPCAprojection);
  std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() <<
  std::endl; std::cerr << std::endl << "EigenValues: " << pca.getEigenValues()
  << std::endl;
  // In this case, pca.getEigenVectors() gives similar eigenVectors to
  eigenVectorsPCA.
  */
  // Transform the original cloud to the origin where the principal components
  // correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) =
      -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(pc_copy, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  // Final transform
  const Eigen::Quaternionf bboxQuaternion(
      eigenVectorsPCA); // Quaternions are a way to do rotations
                        // https://www.youtube.com/watch?v=mHVwd8gYLnI
  Eigen::Vector3f bboxTransform =
      eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  // This viewer has 4 windows, but is only showing images in one of them as
  // written here.
  // pcl::visualization::PCLVisualizer *visu;
  // visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
  // int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
  // visu->createViewPort(0.0, 0.5, 0.5, 1.0, mesh_vp_1);
  // visu->createViewPort(0.5, 0.5, 1.0, 1.0, mesh_vp_2);
  // visu->createViewPort(0.0, 0, 0.5, 0.5, mesh_vp_3);
  // visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
  // visu->addPointCloud(cloudSegmented,
  //                     ColorHandlerXYZ(cloudSegmented, 30, 144, 255),
  //                     "bboxedCloud", mesh_vp_3);
  // visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x,
  //               maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox",
  //               mesh_vp_3);

  if (minPoint.x == 0.0) {
    minPoint.x = Z_min;
    maxPoint.x = Z_max;
  } else if (minPoint.y == 0.0) {
    minPoint.y = Z_min;
    maxPoint.y = Z_max;
  } else if (minPoint.z == 0.0) {
    minPoint.z = Z_min;
    maxPoint.z = Z_max;
  } else {
    std::cout << "Err in computing bbox !!!" << std::endl;
  }
  bboxTransform[2] = (Z_min + Z_max) / 2.0;

  pc_bbox_res bbox;
  bbox.bboxTransform = bboxTransform;
  bbox.bboxQuaternion = bboxQuaternion;
  bbox.minPoint = minPoint;
  bbox.maxPoint = maxPoint;

  return bbox;
}

double time_now() {
  auto time = std::chrono::system_clock::now().time_since_epoch();
  std::chrono::seconds seconds =
      std::chrono::duration_cast<std::chrono::seconds>(time);
  std::chrono::milliseconds ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(time);
  return (double)seconds.count() + ((double)(ms.count() % 1000) / 1000.0);
}

}  // namespace scbot