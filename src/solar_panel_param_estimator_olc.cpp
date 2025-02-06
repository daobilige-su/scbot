#include "solar_panel_param_estimator.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace scbot {

solarPanelParamEstimator::solarPanelParamEstimator() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_pc_transformed_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pc_sp_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pc_ptr_ = empty_pc_ptr;
  pc_transformed_ptr_ = empty_pc_transformed_ptr;
  pc_sp_ptr_ = empty_pc_sp_ptr;
  // pcl_viewer_ = 
}

const ObstaclePanel solarPanelParamEstimator::getPanelParam() {
  // do something here
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

  Eigen::Vector3f coef_plane_norm_ref(0, -0.707, 0.707);
  float coef_plane_norm_diff_thr = 0.2;

  if (pc_ptr_ != nullptr) {
    // transform to robot coord
    // std::cout<<transform_<<std::endl;
    pcl::transformPointCloud(*pc_ptr_, *pc_transformed_ptr_, transform_);

    // passthrough filter
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pc_transformed_ptr_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(pc_z_limit_min, pc_z_limit_max);
    pass.filter(*pc_transformed_ptr_);
    pass.setInputCloud(pc_transformed_ptr_);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(pc_y_limit_min, pc_y_limit_max);
    pass.filter(*pc_transformed_ptr_);
    pass.setInputCloud(pc_transformed_ptr_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(pc_x_limit_min, pc_x_limit_max);
    pass.filter(*pc_transformed_ptr_);

    pass.setInputCloud(pc_transformed_ptr_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(pc_robot_frame_x_min, pc_robot_frame_x_max);
    pass.setNegative (true);
    pass.filter(*pc_transformed_ptr_);

    pcl::copyPointCloud(*pc_transformed_ptr_, *pc_sp_ptr_);

    // extract plane model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
    seg.setOptimizeCoefficients(true); // Optional
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    // seg.setDistanceThreshold(plane_seg_dist_thr);
    pcl::ExtractIndices<pcl::PointXYZ> extract; // Create the filtering object
    seg.setInputCloud(pc_sp_ptr_);

    // auto axis = seg.getAxis();
    // auto eps_ang = seg.getEpsAngle();
    // std::cout << "seg axis: " << axis << std::endl;
    // std::cout << "eps_ange: " << eps_ang << std::endl;
    // seg.setAxis(Eigen::Vector3f(0, -0.707, 0.707));
    // seg.setEpsAngle(M_PI / 12);
    // axis = seg.getAxis();
    // eps_ang = seg.getEpsAngle();
    // std::cout << "seg axis: " << axis << std::endl;
    // std::cout << "eps_ange: " << eps_ang << std::endl;
    auto dist_thr = seg.getDistanceThreshold();
    std::cout << "dist_thr: " << dist_thr << std::endl;
    seg.setDistanceThreshold(plane_seg_dist_thr);
    dist_thr = seg.getDistanceThreshold();
    std::cout << "dist_thr: " << dist_thr << std::endl;

    seg.segment(*inliers, *coefficients);
    if (coefficients->values[1] > 0) {
      coefficients->values[0] = coefficients->values[0] * (-1);
      coefficients->values[1] = coefficients->values[1] * (-1);
      coefficients->values[2] = coefficients->values[2] * (-1);
      coefficients->values[3] = coefficients->values[3] * (-1);
    }
    if (inliers->indices.size() == 0){
      std::cout << "Could not estimate a planar model for the given dataset."
                << std::endl;
    }

    Eigen::Vector3f coef_plane_norm(coefficients->values[0],
                                    coefficients->values[1],
                                    coefficients->values[2]);
    // if (coef_plane_norm(1) > 0) {
    //   coef_plane_norm = -1*coef_plane_norm;
    // }

      Eigen::Vector3f coef_plane_norm_diff =
          (coef_plane_norm - coef_plane_norm_ref);
    if (coef_plane_norm_diff.norm() > coef_plane_norm_diff_thr) {
      std::cout << "solar_panel_param_estimator: plane norm don't satisfy. "
                   "skipping ..."
                << std::endl;
      pc_sp_ptr_->clear();
      return panel_param_;
    }

    // Extract the inliers
    extract.setInputCloud(pc_sp_ptr_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*pc_sp_ptr_);
    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " " << coefficients->values[2]
              << " " << coefficients->values[3] << std::endl;
    std::cout << "Model inliers: " << inliers->indices.size() << std::endl;

    panel_param_.Panel_A = coefficients->values[0];
    panel_param_.Panel_B = coefficients->values[1];
    panel_param_.Panel_C = coefficients->values[2];
    panel_param_.Panel_D = coefficients->values[3];

    // divide pc_sp along X axis into different sub pc
    pcl::PassThrough<pcl::PointXYZ> sub_pc_pass;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sub_pc;
    int sub_pc_num = int((10 - (-10)) / 0.2);
    // sub_pc.resize(int((10 - (-10)) / 0.2));
    float x_min, x_max;
    Eigen::VectorXf sub_pc_count(sub_pc_num);

    for (int i = 0; i < sub_pc_num; ++i) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc_empty(
      new pcl::PointCloud<pcl::PointXYZ>());
      sub_pc.emplace_back(sub_pc_empty);

      x_min = -10+0.2*i;
      x_max = -10 + 0.2 * (i + 1);

      sub_pc_pass.setInputCloud(pc_sp_ptr_);
      sub_pc_pass.setFilterFieldName("x");
      sub_pc_pass.setFilterLimits(x_min, x_max);
      sub_pc_pass.filter(*sub_pc[i]);

      sub_pc_count(i) = sub_pc[i]->size();
    }
    // std::cout << "sub_pc_count" << std::endl;
    // std::cout << sub_pc_count << std::endl;

    int panel_exist_idx = -1;
    for (int i = int(sub_pc_num/2); i < sub_pc_num; ++i) {
      if (sub_pc_count(i) > 50) {
        panel_exist_idx = i;
        break;
      }
    }
    if (panel_exist_idx == -1) {
      std::cout << "isPanelExist: " << panel_param_.isPanelExist
                << ", skipping ..."<< std::endl;
      return panel_param_;
    }

    // if the front visible region has sufficient points, assume there is panel
    // under the robot
    panel_param_.isPanelExist = true;
    // if ((int(sub_pc_num / 2) + int(0.5 / 0.2)) > 50) {
    //   panel_exist_idx = int(sub_pc_num / 2);
    // }

    if (panel_exist_idx == (int(sub_pc_num / 2) + int(0.5 / 0.2))) {
      panel_param_.dis2panel = 0;
    } else {
      panel_param_.dis2panel = 10;
      for (int i = 0; i < int(sub_pc[panel_exist_idx]->size()); ++i) {
        if (sub_pc[panel_exist_idx]->points[i].x < panel_param_.dis2panel) {
          panel_param_.dis2panel = sub_pc[panel_exist_idx]->points[i].x;
        }
      }
    }
    std::cout << "panel_param_.isPanelExist: " << panel_param_.isPanelExist
              << std::endl;
    std::cout << "panel_param_.dis2panel: " << panel_param_.dis2panel
              << std::endl;

    // estimate line param
    Eigen::VectorXf can_z(5);
    can_z << -1, -1, -1, -1, -1;
    Eigen::VectorXi can_z_valid(5);
    can_z_valid << 0, 0, 0, 0, 0;
    Eigen::MatrixXf can_pts(3, 5);
    float can_z_ref = -0.55, can_z_ref_thr = 0.5;
    for (int i = 0; i < 5; ++i) {
      for (const auto pts : sub_pc[panel_exist_idx + 1 + i]->points) {
        if (pts.z > can_z(i)) {
          can_z(i) = pts.z;
          can_pts.block(0, i, 3, 1) << pts.x, pts.y, pts.z;
          if ((can_z_ref - can_z_ref_thr) < can_z(i) &&
              can_z(i) < (can_z_ref + can_z_ref_thr)) {
            can_z_valid(i) = 1;
          }
        }
      }
    }
    std::cout << "can_z: " << std::endl << can_z << std::endl;
    std::cout << "can_pts: " << std::endl << can_pts << std::endl;
    std::cout << "can_z_valid: " << std::endl << can_z_valid << std::endl;

    int left_idx = -1, right_idx = -1;
    for (int i = 0; i < 5; ++i) {
      if (can_z_valid(i)) {
        left_idx = i;
        break;
      }
    }
    for (int i = 0; i < 5; ++i) {
      if (can_z_valid(4-i)) {
        right_idx = 4-i;
        break;
      }
    }
    // if (right_idx > left_idx && abs(can_z(left_idx) - can_z(right_idx)) < 0.5) {
    if (left_idx==-1 || right_idx==-1 || right_idx <= left_idx) {
      std::cout << "no valid line found, skipping ..." << std::endl;
      return panel_param_;
    }
    Eigen::MatrixXf line1_pts(3, 2);
    line1_pts.block(0, 0, 3, 1) << can_pts(0, left_idx), can_pts(1, left_idx),
        can_pts(2, left_idx);
    line1_pts.block(0, 1, 3, 1) << can_pts(0, right_idx), can_pts(1, right_idx),
        can_pts(2, right_idx);
    std::cout << "line1_pts: " << std::endl << line1_pts << std::endl;

    Eigen::Vector3f pts1(line1_pts(0, 0), line1_pts(1, 0), line1_pts(2, 0));
    Eigen::Vector3f pts2(line1_pts(0, 1), line1_pts(1, 1), line1_pts(2, 1));

    // fit a line using the projected valid can_z pts.
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_can_pc(
        (new pcl::PointCloud<pcl::PointXYZ>));
    for (int i = 0; i < 5; ++i) {
      if (can_z_valid(i)) {
        pcl::PointXYZ tmp_pt;
        tmp_pt.x = can_pts(0, i);
        tmp_pt.y = can_pts(1, i);
        tmp_pt.z = can_pts(2, i);
        line_can_pc->points.push_back(tmp_pt);
      }
    }
    if (line_can_pc->points.size() < 2) {
      std::cout << "Not enough candidates to fit the top line, skipping ..."
                << std::endl;
      return panel_param_;
    }

    // std::cout << "line_can_pc: " << std::endl;
    // for (const auto &point : *line_can_pc){
    //   std::cout << "    " << point.x << " " << point.y << " " << point.z
    //             << std::endl;
    // }

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // coefficients->values.resize(4);
    // coefficients->values[0] = panel_param_.Panel_A;
    // coefficients->values[1] = panel_param_.Panel_B;
    // coefficients->values[2] = panel_param_.Panel_C;
    // coefficients->values[3] = panel_param_.Panel_D;

    // Create the filtering object

    pcl::PointCloud<pcl::PointXYZ>::Ptr line_can_pc_proj(
        (new pcl::PointCloud<pcl::PointXYZ>));
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(line_can_pc);
    proj.setModelCoefficients(coefficients);
    proj.filter(*line_can_pc_proj);

    std::cout << "line_can_pc_proj: " << std::endl;
    for (const auto &point : *line_can_pc_proj) {
      std::cout << "    " << point.x << " " << point.y << " " << point.z
                << std::endl;
    }

    // find two pts that the corresponding line's z is over all other pts
    Eigen::Vector3f opt_pts1, opt_pts2;
    bool opt_pts_obtained = false;
    for (int i = 0; i < (int(line_can_pc_proj->size()) - 1); ++i) {
      for (int j = i + 1; j < int(line_can_pc_proj->size()); ++j) {
        Eigen::Vector3f p1(line_can_pc_proj->points[i].x,
                           line_can_pc_proj->points[i].y,
                           line_can_pc_proj->points[i].z);
        Eigen::Vector3f p2(line_can_pc_proj->points[j].x,
                           line_can_pc_proj->points[j].y,
                           line_can_pc_proj->points[j].z);
        bool is_opt = true;
        Eigen::Vector3f sh_vec = p2 - p1;
        for (int k = 0; k < int(line_can_pc_proj->size()); ++k) {
          if (k != i && k != j) {
            Eigen::Vector3f po(line_can_pc_proj->points[k].x,
                               line_can_pc_proj->points[k].y,
                               line_can_pc_proj->points[k].z);
            float sh = (po(0) - p1(0)) / sh_vec(0);
            float z_opt = p1(2) + sh_vec(2) * sh;
            if (z_opt < po(2)) {
              is_opt = false;
              break;
            }
          }
        }
        if (is_opt) {
          float sh1 = (0 - p1(0)) / sh_vec(0);
          float sh2 = (10 - p1(0)) / sh_vec(0);
          opt_pts1 = p1 + sh_vec * sh1;
          opt_pts2 = p2 + sh_vec * sh2;
          opt_pts_obtained = true;
        }
        if (opt_pts_obtained) {
          break;
        }
      }
      if (opt_pts_obtained) {
        break;
      }
    }

    Eigen::Vector3f sac_view_line_pts1(opt_pts1);
    Eigen::Vector3f sac_view_line_pts2(opt_pts2);
    Eigen::Vector3f sac_line_vec((opt_pts2 - opt_pts1) /
                                 (opt_pts2 - opt_pts1).norm());

    // use ransac to find the top line
    // std::vector<int> top_line_inliers;
    // Eigen::VectorXf top_line_coefficients;
    // pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
    //     new pcl::SampleConsensusModelLine<pcl::PointXYZ>(line_can_pc_proj));
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
    // Eigen::Vector3f sac_line_vec(top_line_coefficients(3),
    //                              top_line_coefficients(4),
    //                              top_line_coefficients(5));
    // float sac_anchor_shift = (0 - sac_pts1(0)) / sac_line_vec(0);


    // Eigen::Vector3f sac_view_line_pts1 = sac_pts1 + sac_anchor_shift * sac_line_vec;
    // sac_anchor_shift = (10 - sac_pts1(0)) / sac_line_vec(0);
    // Eigen::Vector3f sac_view_line_pts2 = sac_pts1 + sac_anchor_shift * sac_line_vec;

    

    std::cout << "sac_view_line_pts1: " << std::endl
              << sac_view_line_pts1 << std::endl;
    std::cout << "sac_view_line_pts2: " << std::endl
              << sac_view_line_pts2 << std::endl;

    pcl::PointXYZ sac_view_line_pts1_pcl;
    sac_view_line_pts1_pcl.x = sac_view_line_pts1(0);
    sac_view_line_pts1_pcl.y = sac_view_line_pts1(1);
    sac_view_line_pts1_pcl.z = sac_view_line_pts1(2);
    pcl::PointXYZ sac_view_line_pts2_pcl;
    sac_view_line_pts2_pcl.x = sac_view_line_pts2(0);
    sac_view_line_pts2_pcl.y = sac_view_line_pts2(1);
    sac_view_line_pts2_pcl.z = sac_view_line_pts2(2);

    // Eigen::Vector3f line_vec = pts2 - pts1;
    // float anchor_shift = (0 - pts1(0)) / line_vec(0);
    // Eigen::Vector3f view_line_pts1 = pts1 + anchor_shift * line_vec;
    // anchor_shift = (10 - pts1(0)) / line_vec(0);
    // Eigen::Vector3f view_line_pts2 = pts1 + anchor_shift * line_vec;

    // std::cout << "view_line_pts1: " << std::endl << view_line_pts1 << std::endl;
    // std::cout << "view_line_pts2: " << std::endl << view_line_pts2 << std::endl;

    // pcl::PointXYZ view_line_can_pts1_pcl;
    // view_line_can_pts1_pcl.x = pts1(0);
    // view_line_can_pts1_pcl.y = pts1(1);
    // view_line_can_pts1_pcl.z = pts1(2);
    // pcl::PointXYZ view_line_can_pts2_pcl;
    // view_line_can_pts2_pcl.x = pts2(0);
    // view_line_can_pts2_pcl.y = pts2(1);
    // view_line_can_pts2_pcl.z = pts2(2);
    // pcl::PointXYZ view_line_pts1_pcl;
    // view_line_pts1_pcl.x = view_line_pts1(0);
    // view_line_pts1_pcl.y = view_line_pts1(1);
    // view_line_pts1_pcl.z = view_line_pts1(2);
    // pcl::PointXYZ view_line_pts2_pcl;
    // view_line_pts2_pcl.x = view_line_pts2(0);
    // view_line_pts2_pcl.y = view_line_pts2(1);
    // view_line_pts2_pcl.z = view_line_pts2(2);

    Eigen::Vector3f plane_norm(panel_param_.Panel_A, panel_param_.Panel_B,
                               panel_param_.Panel_C);
    Eigen::Vector3f lines_norm = sac_line_vec.cross(plane_norm);

    float panel_width = 4.0;

    // Eigen::Vector3f bot_line_pt0 = sac_view_line_pts1;
    Eigen::Vector3f bot_line_pt1 =
        sac_view_line_pts1 + panel_width * lines_norm;
    Eigen::Vector3f bot_line_pt2 = bot_line_pt1 + 10.0 * sac_line_vec;
    pcl::PointXYZ sac_view_bot_line_pts1_pcl;
    sac_view_bot_line_pts1_pcl.x = bot_line_pt1(0);
    sac_view_bot_line_pts1_pcl.y = bot_line_pt1(1);
    sac_view_bot_line_pts1_pcl.z = bot_line_pt1(2);
    pcl::PointXYZ sac_view_bot_line_pts2_pcl;
    sac_view_bot_line_pts2_pcl.x = bot_line_pt2(0);
    sac_view_bot_line_pts2_pcl.y = bot_line_pt2(1);
    sac_view_bot_line_pts2_pcl.z = bot_line_pt2(2);


    // find gap
    // pc_sp_ptr_
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sp_transformed_ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f sp_transform;
    sp_transform <<
        sac_line_vec(0), -lines_norm(0), plane_norm(0), sac_view_line_pts1(0),
        sac_line_vec(1), -lines_norm(1), plane_norm(1), sac_view_line_pts1(1), 
        sac_line_vec(2), -lines_norm(2), plane_norm(2), sac_view_line_pts1(2), 
        0, 0, 0, 1;
    // std::cout << "sp_transform:" << std::endl;
    // std::cout << sp_transform << std::endl;
    // std::cout << sp_transform.inverse() << std::endl;
    // std::cout << sp_transform << std::endl;
    Eigen::Matrix4f tmp = sp_transform.inverse();
    sp_transform = tmp;

    pcl::transformPointCloud(*pc_sp_ptr_, *pc_sp_transformed_ptr, sp_transform);
    // pcl::transformPointCloud(*pc_ptr_, *pc_transformed_ptr_, transform_);

    pcl::PassThrough<pcl::PointXYZ> pass_gap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(
        new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<int> gap_pts_count;
    std::vector<float> gap_pts_x;
    std::cout << "gap_pts_count: " << std::endl;
        for (float x = 0.7; x < 5; x = x + 0.02) {
      pass_gap.setInputCloud(pc_sp_transformed_ptr);
      pass_gap.setFilterFieldName("x");
      pass_gap.setFilterLimits(x - 0.02, x + 0.02);
      pass_gap.filter(*tmp_pc);
      pass_gap.setInputCloud(tmp_pc);
      pass_gap.setFilterFieldName("y");
      pass_gap.setFilterLimits(-(panel_width / 2) - 1,
                               -(panel_width / 2) + 1);
      pass_gap.filter(*tmp_pc);
      gap_pts_count.emplace_back(tmp_pc->size());
      gap_pts_x.emplace_back(x);
      std::cout << tmp_pc->size() << ",";
    }
    std::cout << std::endl;

    bool gap_exist = false;
    float gap_x = 0;
    for (int i = 0; i < int(gap_pts_count.size())-5; ++i) {
      if (gap_pts_count[i] == 0) {
        if (gap_pts_count[i + 1] == 0 && gap_pts_count[i + 2] == 0 &&
            gap_pts_count[i + 3] == 0 && gap_pts_count[i + 4] == 0) {
          gap_exist = true;
          gap_x = gap_pts_x[i];
          break;
        }
      }
    }
    std::cout << "gap_exist" << gap_exist << std::endl;

    Eigen::Vector3f gap_pt1, gap_pt2;
    Eigen::Vector3f gap_proj_pt2;
    
    if(gap_exist){
      gap_pt1 =
          sac_view_line_pts1 + panel_width / 2.0 * lines_norm;
      gap_pt2 = gap_pt1 + sac_line_vec * gap_x;
      gap_proj_pt2 << gap_x, -panel_width / 2.0, 0;
    }
    pcl::PointXYZ view_gap_pt1_pcl;
    view_gap_pt1_pcl.x = gap_pt1(0);
    view_gap_pt1_pcl.y = gap_pt1(1);
    view_gap_pt1_pcl.z = gap_pt1(2);
    pcl::PointXYZ view_gap_pt2_pcl;
    view_gap_pt2_pcl.x = gap_pt2(0);
    view_gap_pt2_pcl.y = gap_pt2(1);
    view_gap_pt2_pcl.z = gap_pt2(2);
    pcl::PointXYZ view_gap_proj_pt2_pcl;
    view_gap_proj_pt2_pcl.x = gap_proj_pt2(0);
    view_gap_proj_pt2_pcl.y = gap_proj_pt2(1);
    view_gap_proj_pt2_pcl.z = gap_proj_pt2(2);

    {
      const std::lock_guard<std::mutex> lock(*pcl_viewer_mutex_ptr_);
      pcl_viewer_->removeShape("sac_line");
      pcl_viewer_->removeShape("sac_sphere1");
      pcl_viewer_->removeShape("sac_sphere2");
      pcl_viewer_->removeShape("sac_top_bot_line");
      pcl_viewer_->removeShape("sac_bot_line");
      pcl_viewer_->removeShape("sac_bot_sphere1");
      pcl_viewer_->removeShape("sac_bot_sphere2");
      pcl_viewer_->removePointCloud("pc_sp_transformed");
      pcl_viewer_->removeShape("gap_line");
      pcl_viewer_->removeShape("gap_sphere1");
      pcl_viewer_->removeShape("gap_sphere2");
      pcl_viewer_->removeShape("gap_proj_sphere2");
      pcl_viewer_->removeShape("cube");
      // pcl_viewer_->removeShape("line");
      // pcl_viewer_->removeShape("sphere1");
      // pcl_viewer_->removeShape("sphere2");
      // pcl_viewer_->removeShape("can_sphere1");
      // pcl_viewer_->removeShape("can_sphere2");
      pcl_viewer_->addLine<pcl::PointXYZ>(
          sac_view_line_pts1_pcl, sac_view_line_pts2_pcl, 0, 0, 1, "sac_line");
      pcl_viewer_->addSphere(sac_view_line_pts1_pcl, 0.2, 0.5, 0.5, 0.0, "sac_sphere1"); // radius, r,g,b
      pcl_viewer_->addSphere(sac_view_line_pts2_pcl, 0.2, 0.5, 0.5, 0.0,
                             "sac_sphere2");
      pcl_viewer_->addLine<pcl::PointXYZ>(sac_view_bot_line_pts1_pcl,
                                          sac_view_line_pts1_pcl, 0, 0, 1,
                                          "sac_top_bot_line");
      pcl_viewer_->addLine<pcl::PointXYZ>(sac_view_bot_line_pts1_pcl,
                                          sac_view_bot_line_pts2_pcl, 0, 0, 1,
                                          "sac_bot_line");
      pcl_viewer_->addSphere(sac_view_bot_line_pts1_pcl, 0.2, 0.5, 0.5, 0.0,
                             "sac_bot_sphere1"); // radius, r,g,b
      pcl_viewer_->addSphere(sac_view_bot_line_pts2_pcl, 0.2, 0.5, 0.5, 0.0,
                             "sac_bot_sphere2");
      // pcl_viewer_->addLine<pcl::PointXYZ>(view_line_pts1_pcl,
      //                                     view_line_pts2_pcl, "line");
      // pcl_viewer_->addSphere(view_line_pts1_pcl, 0.2, 0.5, 0.5, 0.0,
      // "sphere1"); // radius, r,g,b pcl_viewer_->addSphere(view_line_pts2_pcl,
      // 0.2, 0.5, 0.5, 0.0, "sphere2");
      // pcl_viewer_->addSphere(view_line_can_pts1_pcl, 0.1, 1.0, 0, 0.0,
      // "can_sphere1"); // radius, r,g,b
      // pcl_viewer_->addSphere(view_line_can_pts2_pcl, 0.1, 1.0, 0, 0.0,
      // "can_sphere2");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          pc_sp_transformed_color_handle(pc_sp_transformed_ptr, 0, 255, 0);
      pcl_viewer_->addPointCloud<pcl::PointXYZ>(pc_sp_transformed_ptr,
                                                pc_sp_transformed_color_handle,
                                                "pc_sp_transformed");
      if(gap_exist){
        pcl_viewer_->addLine<pcl::PointXYZ>(view_gap_pt1_pcl, view_gap_pt2_pcl, 0,
                                            1, 0, "gap_line");
        pcl_viewer_->addSphere(view_gap_pt1_pcl, 0.2, 0, 0.5, 0.0,
                              "gap_sphere1"); // radius, r,g,b
        pcl_viewer_->addSphere(view_gap_pt2_pcl, 0.2, 0, 0.5, 0.0,
                               "gap_sphere2");
        pcl_viewer_->addSphere(view_gap_proj_pt2_pcl, 0.2, 0, 0.5, 0.0,
                               "gap_proj_sphere2");
        pcl_viewer_->addCube(gap_x - 0.02, gap_x + 0.02, -(panel_width / 2) - 1,
                             -(panel_width / 2) + 1, -0.02, 0.02, 0.5, 0, 0, "cube");
      }
    }

  } else {
    std::cout
        << "Warning in solar_panel_param_estimator: Point Could Ptr is NullPtr"
        << std::endl;
  }

  return panel_param_;
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr
solarPanelParamEstimator::getPcTransform() const {
  return pc_transformed_ptr_;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr
solarPanelParamEstimator::getPcSp() const {
  return pc_sp_ptr_;
}

bool solarPanelParamEstimator::setPc(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr) {
  pc_ptr_ = ptr;

  return true;
}

bool solarPanelParamEstimator::setViewer(
    std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer_ptr) {
  pcl_viewer_= viewer_ptr;

  return true; // if you forget to return true, will get a "illegal instruction"
               // error!!! which lead you to think check share_ptr usage, but
               // that is actually right.
}

bool solarPanelParamEstimator::setViewerMutex(std::shared_ptr<std::mutex> viewer_mutex_ptr) {
  pcl_viewer_mutex_ptr_ = viewer_mutex_ptr;

  return true;
}

// bool solarPanelParamEstimator::setDummy(std::shared_ptr<int> du) {
//   dummy_ = du;

//   return true;
// }

bool solarPanelParamEstimator::setTransform(
    std::vector<float> pose3d_trans_ypr) {

  Eigen::VectorXf trans_euler(6);
  trans_euler << pose3d_trans_ypr[0], pose3d_trans_ypr[1], pose3d_trans_ypr[2],
      pose3d_trans_ypr[3], pose3d_trans_ypr[4], pose3d_trans_ypr[5];
  
  poseTransformer::poseTransformer transformer;
  transform_ =  transformer.pose3dEulerToM(trans_euler);
  
  return true;
}
}