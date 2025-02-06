#include "solar_panel_param_estimator.hpp"
#include "scbot_objects.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

namespace scbot
{
// utility functions declarations
pcl::PointCloud<pcl::PointXYZ>::Ptr
ExtractPcByXYZLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, float x_min,
                     float x_max, float y_min, float y_max, float z_min,
                     float z_max);
void PrintPanelParam(const ObstaclePanel& panel_param);

// class member function definitions
solarPanelParamEstimator::solarPanelParamEstimator()
{
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

const ObstaclePanel solarPanelParamEstimator::getPanelParam()
{
  // params
  float pc_x_limit_min = -10.0;
  float pc_x_limit_max = 10.0;
  float pc_y_limit_min = -3.0;
  float pc_y_limit_max = 3.0;
  float pc_z_limit_min = -4.0;
  float pc_z_limit_max = 2.0;

  float pc_robot_frame_x_min = -0.5;
  float pc_robot_frame_x_max = 0.5;

  float plane_seg_dist_thr = 0.05;

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

  // reset panel_param_
  panel_param_ = ObstaclePanel();
  // return if raw pc is not set.
  if (pc_raw_ptr_ == nullptr)
  {
    std::cout << "Warning: in solar_panel_param_estimator: Raw Point Could Ptr "
                 "is NullPtr, skipping ..."
              << std::endl;
    return panel_param_;
  }

  // (1) transform pc to robot frame
  if (debug_verbose_)
  {
    std::cout << "transform_lidar_robot_: " << std::endl;
    std::cout << transform_lidar_robot_ << std::endl;
  }
  pcl::transformPointCloud(*pc_raw_ptr_, *pc_robot_transformed_ptr_,
                           transform_lidar_robot_);

  // (2) Extract pc close to robot, and exclude frames of the robot
  pc_roi_ptr_ = ExtractPcByXYZLimits(
      pc_robot_transformed_ptr_, pc_x_limit_min, pc_x_limit_max, pc_y_limit_min,
      pc_y_limit_max, pc_z_limit_min, pc_z_limit_max);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pc_roi_ptr_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(pc_robot_frame_x_min, pc_robot_frame_x_max);
  pass.setNegative(true);
  pass.filter(*pc_roi_ptr_);
  // visualization
  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);
    pcl_viewer_->removePointCloud("pc_robot_transformed");
    pcl_viewer_->removePointCloud("pc_roi");
    if (debug_verbose_) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_robot_transformed_color_handle(pc_robot_transformed_ptr_, 70, 70,
                                            70);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(
          pc_robot_transformed_ptr_, pc_robot_transformed_color_handle,
          "pc_robot_transformed");
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        pc_roi_color_handle(pc_roi_ptr_, 255, 255, 255);
    pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_roi_ptr_, pc_roi_color_handle,
                                              "pc_roi");
  }

  // (3) extract solar panel plane model
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  pcl::ExtractIndices<pcl::PointXYZ> extract;  // Create the filtering object
  seg.setInputCloud(pc_roi_ptr_);
  seg.setDistanceThreshold(plane_seg_dist_thr);
  seg.segment(*inliers, *coefficients);
  // if (coefficients->values[1] > 0)
  // {  // fix the plane norm to one dir
  //   coefficients->values[0] = coefficients->values[0] * (-1);
  //   coefficients->values[1] = coefficients->values[1] * (-1);
  //   coefficients->values[2] = coefficients->values[2] * (-1);
  //   coefficients->values[3] = coefficients->values[3] * (-1);
  // }
  if (debug_verbose_)
  {
    std::cout << "Plane Model coefficients: " << coefficients->values[0] << ", "
              << coefficients->values[1] << ", " << coefficients->values[2]
              << ", " << coefficients->values[3] << std::endl;
    std::cout << "Plane Model inliers: " << inliers->indices.size()
              << std::endl;
  }
  // check if there are enough number of inliers
  if (inliers->indices.size() == 0)
  {
    std::cout << "Warning: in solar_panel_param_estimator: Could not estimate "
                 "a planar model for the given dataset. skipping ..."
              << std::endl;
    return panel_param_;
  }

  // check with plane norm with pre-defined reference
  Eigen::Vector3f plane_norm(coefficients->values[0],
                                  coefficients->values[1],
                                  coefficients->values[2]);
  Eigen::Vector3f plane_norm_diff = (plane_norm - sp_plane_norm_ref);
  if (plane_norm_diff.norm() > sp_plane_norm_diff_thr)
  {
    std::cout << "Warning: in solar_panel_param_estimator: plane norm don't "
                 "satisfy. skipping ..."
              << std::endl;
    return panel_param_;
  }

  // Extract the inliers pc to be pc_sp_ptr_
  extract.setInputCloud(pc_roi_ptr_);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*pc_sp_ptr_);
  panel_param_.Panel_A = coefficients->values[0];
  panel_param_.Panel_B = coefficients->values[1];
  panel_param_.Panel_C = coefficients->values[2];
  panel_param_.Panel_D = coefficients->values[3];

  // visualization
  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);
    pcl_viewer_->removePointCloud("pc_sp");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        pc_sp_color_handle(pc_sp_ptr_, 255, 0, 0);
    pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_sp_ptr_, pc_sp_color_handle,
                                              "pc_sp");
  }

  // (4) Detect Solar Panels within 10m along X direction
  // divide pc_sp along X axis into different sub pc
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sub_pc;

  pcl::PassThrough<pcl::PointXYZ> sub_pc_pass;
  int sub_pc_num = int((sp_det_range - (-sp_det_range)) / sub_pc_x_size);
  float x_min, x_max;
  Eigen::VectorXf sub_pc_count(sub_pc_num);
  for (int i = 0; i < sub_pc_num; ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc_empty(
        new pcl::PointCloud<pcl::PointXYZ>());
    sub_pc.emplace_back(sub_pc_empty);
    x_min = -sp_det_range + sub_pc_x_size * i;
    x_max = -sp_det_range + sub_pc_x_size * (i + 1);
    sub_pc_pass.setInputCloud(pc_sp_ptr_);
    sub_pc_pass.setFilterFieldName("x");
    sub_pc_pass.setFilterLimits(x_min, x_max);
    sub_pc_pass.filter(*sub_pc[i]);
    sub_pc_count(i) = sub_pc[i]->size();
  }

  // determine existance of solar panel, along X dir, find the 1st sub_pc with
  // more than sub_pc_min_pts number of points.
  int panel_exist_idx = -1;
  for (int i = int(sub_pc_num / 2); i < sub_pc_num; ++i)
  {
    if (sub_pc_count(i) > sub_pc_min_pts) {
      panel_exist_idx = i;
      break;
    }
  }
  if (panel_exist_idx == -1)
  {
    std::cout << "Warning: in solar_panel_param_estimator: isPanelExist: "
              << panel_param_.isPanelExist << ", skipping ..." << std::endl;
    return panel_param_;
  }
  panel_param_.isPanelExist = true;

  // if the front visible region has sufficient points, assume there is panel
  // under the robot
  if (panel_exist_idx ==
      (int(sub_pc_num / 2) + int(pc_robot_frame_x_max / 0.2))) {
    panel_param_.dis2panel = 0;
  } else {
    panel_param_.dis2panel = sp_det_range;
    // find the point with min x value within the sub_pc
    for (int i = 0; i < int(sub_pc[panel_exist_idx]->size()); ++i)
    {
      if (sub_pc[panel_exist_idx]->points[i].x < panel_param_.dis2panel)
      {
        panel_param_.dis2panel = sub_pc[panel_exist_idx]->points[i].x;
      }
    }
  }
  if(debug_verbose_){
    std::cout << "panel_param_.isPanelExist: " << panel_param_.isPanelExist
              << std::endl;
  std::cout << "panel_param_.dis2panel: " << panel_param_.dis2panel
            << std::endl;
  }

  // (5) estimate top an bottom lines params
  // use the closest 5 sub_pc, and get their pts with highest z
  Eigen::VectorXf can_z(top_line_can_num);
  can_z << -1, -1, -1, -1, -1;
  Eigen::VectorXi can_z_valid(top_line_can_num);
  can_z_valid << 0, 0, 0, 0, 0;
  Eigen::MatrixXf can_pts(3, top_line_can_num);
  for (int i = 0; i < top_line_can_num; ++i) {
    for (const auto pts : sub_pc[panel_exist_idx + 1 + i]->points) {
      if (pts.z > can_z(i)) {
        can_z(i) = pts.z;
        can_pts.block(0, i, 3, 1) << pts.x, pts.y, pts.z;
        if ((sub_pc_top_z_ref - sub_pc_top_z_ref_thr) < can_z(i) &&
            can_z(i) < (sub_pc_top_z_ref + sub_pc_top_z_ref_thr)) {
          can_z_valid(i) = 1;
        }
      }
    }
  }
  if(debug_verbose_){
    std::cout << "can_z: " << std::endl << can_z << std::endl;
    std::cout << "can_pts: " << std::endl << can_pts << std::endl;
    std::cout << "can_z_valid: " << std::endl << can_z_valid << std::endl;
  }
  // project valid candidates on the solar panel plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr top_line_can_pc(
      (new pcl::PointCloud<pcl::PointXYZ>)); // turn can pts to pc
  for (int i = 0; i < top_line_can_num; ++i) {
    if (can_z_valid(i)) {
      pcl::PointXYZ tmp_pt;
      tmp_pt.x = can_pts(0, i);
      tmp_pt.y = can_pts(1, i);
      tmp_pt.z = can_pts(2, i);
      top_line_can_pc->points.push_back(tmp_pt);
    }
  }
  if (top_line_can_pc->points.size() < 2) {
    std::cout << "Not enough candidates to fit the top line, skipping ..."
              << std::endl;
    return panel_param_;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr top_line_can_pc_proj(
      (new pcl::PointCloud<pcl::PointXYZ>)); // projected can pts on plane
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(top_line_can_pc);
  proj.setModelCoefficients(coefficients);
  proj.filter(*top_line_can_pc_proj);
  if (debug_verbose_) {
    std::cout << "top_line_can_pc_proj: " << std::endl;
    for (const auto &point : *top_line_can_pc_proj) {
      std::cout << "    " << point.x << " " << point.y << " " << point.z
                << std::endl;
    }
  }

  // algo-1: fit the top line using the projected valid can_z pts and extract
  // the best 2 points that the line going through these 2 points are above all
  // other points
  // 
  // find two pts that the corresponding line's z is over all other pts
  Eigen::Vector3f opt_pts1, opt_pts2;
  bool opt_pts_obtained = false;
  for (int i = 0; i < (int(top_line_can_pc_proj->size()) - 1); ++i) {
    for (int j = i + 1; j < int(top_line_can_pc_proj->size()); ++j) {
      Eigen::Vector3f p1(top_line_can_pc_proj->points[i].x,
                         top_line_can_pc_proj->points[i].y,
                         top_line_can_pc_proj->points[i].z);
      Eigen::Vector3f p2(top_line_can_pc_proj->points[j].x,
                         top_line_can_pc_proj->points[j].y,
                         top_line_can_pc_proj->points[j].z);
      
      bool is_opt = true;
      Eigen::Vector3f sh_vec = p2 - p1;
      for (int k = 0; k < int(top_line_can_pc_proj->size()); ++k) {
        if (k != i && k != j) {
          Eigen::Vector3f po(top_line_can_pc_proj->points[k].x,
                             top_line_can_pc_proj->points[k].y,
                             top_line_can_pc_proj->points[k].z);
          float sh = (po(0) - p1(0)) / sh_vec(0);
          // z of point on the line wih same x
          float z_opt = p1(2) + sh_vec(2) * sh; 
          if (z_opt < po(2)) {
            is_opt = false;
            break;
          }
        }
      }
      if (is_opt)
      {
        float sh1 = (0 - p1(0)) / sh_vec(0);
        float sh2 = (sp_det_range - p1(0)) / sh_vec(0);
        opt_pts1 = p1 + sh_vec * sh1;  // pt on line with x=0
        opt_pts2 = p2 + sh_vec * sh2;  // pt on line with x=10
        opt_pts_obtained = true;
      }
      if (opt_pts_obtained)
      {
        break;
      }
    }
    if (opt_pts_obtained)
    {
      break;
    }
  }
  // save line's 2 vertex points and its direction vector
  Eigen::Vector3f top_line_pt1(opt_pts1);
  Eigen::Vector3f top_line_pt2(opt_pts2);
  Eigen::Vector3f top_line_vec((opt_pts2 - opt_pts1) /
                               (opt_pts2 - opt_pts1).norm());

  // algo-2: use ransac to find the top line
  // std::vector<int> top_line_inliers;
  // Eigen::VectorXf top_line_coefficients;
  // pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
  //     new
  //     pcl::SampleConsensusModelLine<pcl::PointXYZ>(top_line_can_pc_proj));
  // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
  // ransac.setDistanceThreshold(.01);
  // ransac.computeModel();
  // ransac.getInliers(top_line_inliers);
  // std::cout << "top_line_inliers: " << top_line_inliers[0]
  //           << top_line_inliers[1] << top_line_inliers[2]
  //           << top_line_inliers[3] << top_line_inliers[4] << std::endl;
  // ransac.getModelCoefficients(top_line_coefficients);

  // Eigen::Vector3f sac_pts1(top_line_coefficients(0),
  //                          top_line_coefficients(1),
  //                          top_line_coefficients(2));
  // // Eigen::Vector3f pts2(line1_pts(0, 1), line1_pts(1, 1), line1_pts(2, 1));
  // Eigen::Vector3f top_line_vec(top_line_coefficients(3),
  //                              top_line_coefficients(4),
  //                              top_line_coefficients(5));
  // float sac_anchor_shift = (0 - sac_pts1(0)) / top_line_vec(0);

  // Eigen::Vector3f top_line_pt1 = sac_pts1 + sac_anchor_shift *
  // top_line_vec;
  // sac_anchor_shift = (sp_det_range - sac_pts1(0)) / top_line_vec(0);
  // Eigen::Vector3f top_line_pt2 = sac_pts1 + sac_anchor_shift *
  // top_line_vec;
  
  if(debug_verbose_){
    std::cout << "top_line_pt1: " << std::endl
              << top_line_pt1 << std::endl;
    std::cout << "top_line_pt2: " << std::endl
              << top_line_pt2 << std::endl;
  }

  pcl::PointXYZ top_line_pt1_pcl;
  top_line_pt1_pcl.x = top_line_pt1(0);
  top_line_pt1_pcl.y = top_line_pt1(1);
  top_line_pt1_pcl.z = top_line_pt1(2);
  pcl::PointXYZ top_line_pt2_pcl;
  top_line_pt2_pcl.x = top_line_pt2(0);
  top_line_pt2_pcl.y = top_line_pt2(1);
  top_line_pt2_pcl.z = top_line_pt2(2);

  // (6) find the bottom line
  // get the line orth to top and bot line
  Eigen::Vector3f orth_line_dir_vec = top_line_vec.cross(plane_norm);
  // get bottom line based on panel width
  Eigen::Vector3f bot_line_pt1 =
      top_line_pt1 + panel_width * orth_line_dir_vec;
  Eigen::Vector3f bot_line_pt2 = bot_line_pt1 + sp_det_range * top_line_vec;

  // save lines param
  panel_param_.LINE1_x = top_line_pt1(0);
  panel_param_.LINE1_y = top_line_pt1(1);
  panel_param_.LINE1_z = top_line_pt1(2);
  panel_param_.LINE1_m = top_line_vec(0);
  panel_param_.LINE1_n = top_line_vec(1);
  panel_param_.LINE1_p = top_line_vec(2);
  panel_param_.LINE2_x = bot_line_pt1(0);
  panel_param_.LINE2_y = bot_line_pt1(1);
  panel_param_.LINE2_z = bot_line_pt1(2);
  panel_param_.LINE2_m = top_line_vec(0);
  panel_param_.LINE2_n = top_line_vec(1);
  panel_param_.LINE2_p = top_line_vec(2);

  // visualization
  pcl::PointXYZ bot_line_pt1_pcl;
  bot_line_pt1_pcl.x = bot_line_pt1(0);
  bot_line_pt1_pcl.y = bot_line_pt1(1);
  bot_line_pt1_pcl.z = bot_line_pt1(2);
  pcl::PointXYZ bot_line_pt2_pcl;
  bot_line_pt2_pcl.x = bot_line_pt2(0);
  bot_line_pt2_pcl.y = bot_line_pt2(1);
  bot_line_pt2_pcl.z = bot_line_pt2(2);
  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);

    pcl_viewer_->removeShape("top_line");
    pcl_viewer_->removeShape("top_line_sphere1");
    pcl_viewer_->removeShape("top_line_sphere2");
    pcl_viewer_->removeShape("bot_line");
    pcl_viewer_->removeShape("bot_line_sphere1");
    pcl_viewer_->removeShape("bot_line_sphere2");
    pcl_viewer_->removeShape("orth_line");
    pcl_viewer_->removeShape("orth_line_far");

    pcl_viewer_->addLine<pcl::PointXYZ>(top_line_pt1_pcl, top_line_pt2_pcl, 0,
                                        0, 1, "top_line");
    pcl_viewer_->addSphere(top_line_pt1_pcl, 0.2, 0.5, 0.5, 0.0,
                           "top_line_sphere1"); // radius, r,g,b
    pcl_viewer_->addSphere(top_line_pt2_pcl, 0.2, 0.5, 0.5, 0.0,
                           "top_line_sphere2");
    pcl_viewer_->addLine<pcl::PointXYZ>(bot_line_pt1_pcl, bot_line_pt2_pcl, 0,
                                        0, 1, "bot_line");
    pcl_viewer_->addSphere(bot_line_pt1_pcl, 0.2, 0.5, 0.5, 0.0,
                           "bot_line_sphere1"); // radius, r,g,b
    pcl_viewer_->addSphere(bot_line_pt2_pcl, 0.2, 0.5, 0.5, 0.0,
                           "bot_line_sphere2");
    pcl_viewer_->addLine<pcl::PointXYZ>(bot_line_pt1_pcl, top_line_pt1_pcl, 0,
                                        0, 1, "orth_line");
    pcl_viewer_->addLine<pcl::PointXYZ>(bot_line_pt2_pcl, top_line_pt2_pcl, 0,
                                        0, 1, "orth_line_far");
  }

  // (7) find gap and distance
  // transform pc_sp_ptr_ to solar panel frame
  // with top line as x axis, orth line as y axis
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sp_transformed_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Matrix4f sp_transform;
  sp_transform << top_line_vec(0), -orth_line_dir_vec(0), plane_norm(0), top_line_pt1(0), 
                  top_line_vec(1), -orth_line_dir_vec(1), plane_norm(1), top_line_pt1(1), 
                  top_line_vec(2), -orth_line_dir_vec(2), plane_norm(2), top_line_pt1(2), 
                  0, 0, 0, 1;
  Eigen::Matrix4f tmp = sp_transform.inverse();
  sp_transform = tmp;
  pcl::transformPointCloud(*pc_sp_ptr_, *pc_sp_transformed_ptr, sp_transform);
  // define a narrow long cube, move it along x dir,
  // when there is no points inside cube for continuous times, it's gap
  pcl::PassThrough<pcl::PointXYZ> pass_gap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> gap_pts_count; // pts within the cube
  std::vector<float> gap_pts_x; // mid x coord of cube
  if (debug_verbose_) {
    std::cout << "gap_pts_count: " << std::endl;
  }
  for (float x = pc_robot_frame_x_max + gap_start_shift; x < gap_det_range;
       x = x + gap_det_res) {
    pass_gap.setInputCloud(pc_sp_transformed_ptr);
    pass_gap.setFilterFieldName("x");
    pass_gap.setFilterLimits(x - gap_det_res, x + gap_det_res);
    pass_gap.filter(*tmp_pc);
    pass_gap.setInputCloud(tmp_pc);
    pass_gap.setFilterFieldName("y");
    pass_gap.setFilterLimits(-(panel_width / 2) - 1, -(panel_width / 2) + 1);
    pass_gap.filter(*tmp_pc);
    gap_pts_count.emplace_back(tmp_pc->size());
    gap_pts_x.emplace_back(x);
    if(debug_verbose_){
      std::cout << tmp_pc->size() << ",";
    }
  }
  if(debug_verbose_){
    std::cout << std::endl;
  }
  // gap exist if there are continous 5 cubes are empty
  bool gap_exist = false;
  float gap_x = 0;
  for (int i = 0; i < int(gap_pts_count.size()) - 5; ++i)
  {
    if (gap_pts_count[i] == 0)
    {
      if (gap_pts_count[i + 1] == 0 && gap_pts_count[i + 2] == 0 &&
          gap_pts_count[i + 3] == 0 && gap_pts_count[i + 4] == 0)
      {
        gap_exist = true;
        gap_x = gap_pts_x[i] - gap_det_res; // minimum x of cube
        break;
      }
    }
  }
  if(debug_verbose_){
    std::cout << "gap_exist: " << gap_exist << std::endl;
  }
  // save gap points
  Eigen::Vector3f gap_pt1, gap_pt2;
  // gap_pt2 projected to sp frame, for debuging only
  Eigen::Vector3f gap_pt2_sp_projected; 
  if (gap_exist)
  {
    gap_pt1 = top_line_pt1 + panel_width / 2.0 * orth_line_dir_vec;
    gap_pt2 = gap_pt1 + top_line_vec * gap_x;
    gap_pt2_sp_projected << gap_x, -panel_width / 2.0, 0;
  }

  // visual gap det area
  Eigen::Vector3f gap_det_pt1 = top_line_pt1 + gap_det_range * top_line_vec;
  Eigen::Vector3f gap_det_pt2 = bot_line_pt1 + gap_det_range * top_line_vec;

  // save gap results
  panel_param_.isGapExist = gap_exist;
  panel_param_.dis2gap = gap_x;

  // visualization
  pcl::PointXYZ gap_det_pt1_pcl;
  gap_det_pt1_pcl.x = gap_det_pt1(0);
  gap_det_pt1_pcl.y = gap_det_pt1(1);
  gap_det_pt1_pcl.z = gap_det_pt1(2);
  pcl::PointXYZ gap_det_pt2_pcl;
  gap_det_pt2_pcl.x = gap_det_pt2(0);
  gap_det_pt2_pcl.y = gap_det_pt2(1);
  gap_det_pt2_pcl.z = gap_det_pt2(2);

  pcl::PointXYZ gap_pt1_pcl;
  gap_pt1_pcl.x = gap_pt1(0);
  gap_pt1_pcl.y = gap_pt1(1);
  gap_pt1_pcl.z = gap_pt1(2);
  pcl::PointXYZ gap_pt2_pcl;
  gap_pt2_pcl.x = gap_pt2(0);
  gap_pt2_pcl.y = gap_pt2(1);
  gap_pt2_pcl.z = gap_pt2(2);
  pcl::PointXYZ gap_pt2_sp_projected_pcl;
  gap_pt2_sp_projected_pcl.x = gap_pt2_sp_projected(0);
  gap_pt2_sp_projected_pcl.y = gap_pt2_sp_projected(1);
  gap_pt2_sp_projected_pcl.z = gap_pt2_sp_projected(2);

  {
    const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);

    pcl_viewer_->removeShape("gap_det_line");
    pcl_viewer_->removeShape("gap_det_line_sphere1");
    pcl_viewer_->removeShape("gap_det_line_sphere2");
    pcl_viewer_->removePointCloud("pc_sp_transformed");
    pcl_viewer_->removeShape("gap_line");
    pcl_viewer_->removeShape("gap_line_sphere1");
    pcl_viewer_->removeShape("gap_line_sphere2");
    pcl_viewer_->removeShape("gap_line_proj_sphere2");
    pcl_viewer_->removeShape("gap_cube");

    pcl_viewer_->addLine<pcl::PointXYZ>(gap_det_pt1_pcl, gap_det_pt2_pcl, 0, 1, 0,
                                        "gap_det_line");
    pcl_viewer_->addSphere(gap_det_pt1_pcl, 0.2, 0, 0.5, 0.0,
                           "gap_det_line_sphere1"); // radius, r,g,b
    pcl_viewer_->addSphere(gap_det_pt2_pcl, 0.2, 0, 0.5, 0.0,
                           "gap_det_line_sphere2");
    if (debug_verbose_) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_sp_transformed_color_handle(pc_sp_transformed_ptr, 0, 255, 0);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_sp_transformed_ptr,
                                                pc_sp_transformed_color_handle,
                                                "pc_sp_transformed");
    }
    if (gap_exist) {
      pcl_viewer_->addLine<pcl::PointXYZ>(gap_pt1_pcl, gap_pt2_pcl, 0, 1, 0,
                                          "gap_line");
      pcl_viewer_->addSphere(gap_pt1_pcl, 0.2, 0, 0.5, 0.0,
                             "gap_line_sphere1"); // radius, r,g,b
      pcl_viewer_->addSphere(gap_pt2_pcl, 0.2, 0, 0.5, 0.0, "gap_line_sphere2");
      if (debug_verbose_) {
        pcl_viewer_->addSphere(gap_pt2_sp_projected_pcl, 0.2, 0, 0.5, 0.0,
                               "gap_line_proj_sphere2");
        pcl_viewer_->addCube(gap_x - gap_det_res, gap_x + gap_det_res,
                             -(panel_width / 2) - 1, -(panel_width / 2) + 1,
                             -gap_det_res, gap_det_res, 0.5, 0, 0, "gap_cube");
      }
    }
  }

  PrintPanelParam(panel_param_);
  return panel_param_;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr
solarPanelParamEstimator::getPcTransform() const {
  return pc_robot_transformed_ptr_;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr
solarPanelParamEstimator::getPcSp() const {
  return pc_sp_ptr_;
}

bool solarPanelParamEstimator::setPc(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr) {
  pcl::copyPointCloud(*ptr, *pc_raw_ptr_);  // pcl::PointXYZI to pcl::PointXYZ

  return true;
}

bool solarPanelParamEstimator::setViewer(
    std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_ptr) {
  pcl_viewer_ = viewer_ptr;

  return true;  // if you forget to return true, will get a "illegal instruction"
                // error!!! which lead you to think check share_ptr usage, but
                // that is actually right.
}

bool solarPanelParamEstimator::setViewerMutex(
    std::shared_ptr<std::mutex> viewer_mutex_ptr) {
  pcl_viewer_mutex_ptr_ = viewer_mutex_ptr;

  return true;
}

bool solarPanelParamEstimator::setTransform(
    std::vector<float> pose3d_trans_ypr) {
  Eigen::VectorXf trans_euler(6);
  trans_euler << pose3d_trans_ypr[0], pose3d_trans_ypr[1], pose3d_trans_ypr[2],
      pose3d_trans_ypr[3], pose3d_trans_ypr[4], pose3d_trans_ypr[5];

  poseTransformer::poseTransformer transformer;
  transform_lidar_robot_ = transformer.pose3dEulerToM(trans_euler);

  return true;
}

bool solarPanelParamEstimator::setDebugVerbose(bool flag) {
  debug_verbose_ = flag;

  return true;
}

// utility functions
pcl::PointCloud<pcl::PointXYZ>::Ptr
ExtractPcByXYZLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, float x_min,
                     float x_max, float y_min, float y_max, float z_min,
                     float z_max) {
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

void PrintPanelParam(const ObstaclePanel& panel_param) {
  std::cout << "panel param:" << std::endl;
  std::cout << "{" << std::endl;
  std::cout << "  isPanelExist: " << panel_param.isPanelExist << std::endl;
  std::cout << "  dis2panel: " << panel_param.dis2panel << std::endl;
  std::cout << "  isGapExist: " << panel_param.isGapExist << std::endl;
  std::cout << "  dis2gap: " << panel_param.dis2gap << std::endl;
  std::cout << "  Panel_A: " << panel_param.Panel_A << std::endl;
  std::cout << "  Panel_B: " << panel_param.Panel_B << std::endl;
  std::cout << "  Panel_C: " << panel_param.Panel_C << std::endl;
  std::cout << "  Panel_D: " << panel_param.Panel_D << std::endl;
  std::cout << "  LINE1_x: " << panel_param.LINE1_x << std::endl;
  std::cout << "  LINE1_y: " << panel_param.LINE1_y << std::endl;
  std::cout << "  LINE1_z: " << panel_param.LINE1_z << std::endl;
  std::cout << "  LINE1_m: " << panel_param.LINE1_m << std::endl;
  std::cout << "  LINE1_n: " << panel_param.LINE1_n << std::endl;
  std::cout << "  LINE1_p: " << panel_param.LINE1_p << std::endl;
  std::cout << "  LINE2_x: " << panel_param.LINE2_x << std::endl;
  std::cout << "  LINE2_y: " << panel_param.LINE2_y << std::endl;
  std::cout << "  LINE2_z: " << panel_param.LINE2_z << std::endl;
  std::cout << "  LINE2_m: " << panel_param.LINE2_m << std::endl;
  std::cout << "  LINE2_n: " << panel_param.LINE2_n << std::endl;
  std::cout << "  LINE2_p: " << panel_param.LINE2_p << std::endl;
  std::cout << "}" << std::endl;
}

}  // namespace scbot