#include <memory>
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

#include "solar_panel_param_estimator.hpp"
#include "simple_obstacle_detector.hpp"

using namespace robosense::lidar;
using namespace pcl::visualization;

typedef PointCloudT<PointXYZI> PointCloudMsg;

std::shared_ptr<PCLVisualizer> pcl_viewer;
auto mtx_viewer_ptr = std::make_shared<std::mutex>();

// for simple debug
// std::string pcap_file_path = "../../../bagfiles/scbot/test01.pcap";
// std::string pcap_file_path =
//     "../../../bagfiles/scbot/pcap-20250311-2/pickup_truck_near.pcap";
std::string pcap_file_path =
    "../../../bagfiles/scbot/pcap-20250311-1/people_dynamics.pcap";

bool enable_panel_param_est = false;
bool enable_obs_det = true;
scbot::solarPanelParamEstimator paramEst;
bool param_est_debug_verbose_flag = false;
std::vector<float> top_lidar_pose3d_trans_ypr = {0, 1, 1, -M_PI / 2.0, 0, M_PI};
scbot::SimpleObstacleDetector simObsDet;
bool bbox_in_robot_coord = true;
bool obs_det_debug_verbose_flag = false;
std::vector<float> bottom_lidar_pose3d_trans_ypr = {0, 0, 0, -M_PI / 2.0, 0, M_PI};

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

bool checkKeywordExist(int argc, const char *const *argv, const char *str) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], str) == 0) {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char *const *argv, const char *str,
                   std::string &val) {
  int index = -1;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], str) == 0) {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc) {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char *argv[], RSDriverParam &param) {
  std::string result_str;

  //
  // input param
  //
  parseArgument(argc, argv, "-pcap", param.input_param.pcap_path);

  // debug
  if (!pcap_file_path.empty()) {
    param.input_param.pcap_path = pcap_file_path;
  }

  if (param.input_param.pcap_path.empty()) {
    param.input_type = InputType::ONLINE_LIDAR;
  } else {
    param.input_type = InputType::PCAP_FILE;
  }

  if (parseArgument(argc, argv, "-msop", result_str)) {
    param.input_param.msop_port = std::stoi(result_str);
  }

  if (parseArgument(argc, argv, "-difop", result_str)) {
    param.input_param.difop_port = std::stoi(result_str);
  }

  parseArgument(argc, argv, "-group", param.input_param.group_address);
  parseArgument(argc, argv, "-host", param.input_param.host_address);

  //
  // decoder param
  //
  if (parseArgument(argc, argv, "-type", result_str)) {
    param.lidar_type = strToLidarType(result_str);
  } else {
    param.lidar_type = strToLidarType("RSBP");
  }

  param.decoder_param.wait_for_difop = false;

  if (parseArgument(argc, argv, "-x", result_str)) {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str)) {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str)) {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str)) {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-pitch", result_str)) {
    param.decoder_param.transform_param.pitch = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-yaw", result_str)) {
    param.decoder_param.transform_param.yaw = std::stof(result_str);
  }
}

void printHelpMenu() {
  RS_MSG << "Arguments: " << RS_REND;
  RS_MSG
      << "  -type   = LiDAR type(RS16, RS32, RSBP, RSHELIOS, RS128, RS80, RSM1)"
      << RS_REND;
  RS_MSG << "  -pcap   = The path of the pcap file, off-line mode if it is "
            "true, else online mode."
         << RS_REND;
  RS_MSG << "  -msop   = LiDAR msop port number,the default value is 6699"
         << RS_REND;
  RS_MSG << "  -difop  = LiDAR difop port number,the default value is 7788"
         << RS_REND;
  RS_MSG << "  -group  = LiDAR destination group address if multi-cast mode."
         << RS_REND;
  RS_MSG << "  -host   = Host address." << RS_REND;
  RS_MSG << "  -x      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -y      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -z      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -roll   = Transformation parameter, unit: radian " << RS_REND;
  RS_MSG << "  -pitch  = Transformation parameter, unit: radian " << RS_REND;
  RS_MSG << "  -yaw    = Transformation parameter, unit: radian " << RS_REND;
}

void exceptionCallback(const Error &code) {
  RS_WARNING << code.toString() << RS_REND;
}

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void) {
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL) {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg) {
  stuffed_cloud_queue.push(msg);
}

bool to_exit_process = false;

// process point cloud (will be done in a seperate thread)
void processCloud(void) {
  while (!to_exit_process) {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL) {
      continue;
    }

    //
    // show the point cloud
    //
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl_pointcloud->points.swap(msg->points);
    pcl_pointcloud->height = msg->height;
    pcl_pointcloud->width = msg->width;
    pcl_pointcloud->is_dense = msg->is_dense;

    // compute solar panel params
    if (enable_panel_param_est) {  
      paramEst.setPc(pcl_pointcloud);
      auto panel_param = paramEst.getPanelParam();
    }

    if (enable_obs_det) {
      simObsDet.setPc(pcl_pointcloud);
      auto obs_param = simObsDet.getObstacleObjs();
    }

    free_cloud_queue.push(msg);
  }
}

// This main thread is for pcl viewer.
// the point cloud handling is done in the processCloud thread above.
int main(int argc, char *argv[]) {
  // show RS driver info
  RS_TITLE << "------------------------------------------------------"
           << RS_REND;
  RS_TITLE << "            RS_Driver Viewer Version: v" << getDriverVersion()
           << RS_REND;
  RS_TITLE << "------------------------------------------------------"
           << RS_REND;

  // handle params
  // if (argc < 2) {
  //   printHelpMenu();
  //   return 0;
  // }

  if (checkKeywordExist(argc, argv, "-h") ||
      checkKeywordExist(argc, argv, "--help")) {
    printHelpMenu();
    return 0;
  }

  RSDriverParam param;
  parseParam(argc, argv, param);
  param.print();

  // setup pcl viewer
  pcl_viewer = std::make_shared<PCLVisualizer>("RSPointCloudViewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "rslidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2,
                                               "rslidar");

  pcl_viewer->initCameraParameters();
  pcl_viewer->setCameraPosition(-15, -15, 15,   0, 0, 0,   0, 0, 1);

  // setup RS driver 
  LidarDriver<PointCloudMsg> driver;
  driver.regExceptionCallback(exceptionCallback);
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  if (!driver.init(param)) {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  // setup paramEst
  paramEst.setTransform(top_lidar_pose3d_trans_ypr);
  paramEst.setViewerMutex(mtx_viewer_ptr);
  paramEst.setViewer(pcl_viewer);
  paramEst.setDebugVerbose(param_est_debug_verbose_flag);

  // setup obstacle detector
  simObsDet.setTransform(bottom_lidar_pose3d_trans_ypr);
  simObsDet.setViewerMutex(mtx_viewer_ptr);
  simObsDet.setViewer(pcl_viewer);
  simObsDet.setDebugVerbose(obs_det_debug_verbose_flag);
  simObsDet.setUseRobotCoordInBbox(bbox_in_robot_coord);

  // register thread to process point cloud data
  std::thread cloud_handle_thread = std::thread(processCloud);

  // start working
  RS_INFO << "RoboSense Lidar-Driver Viewer start......" << RS_REND;
  driver.start();

  while (!pcl_viewer->wasStopped()) { // show point cloud in pcl viewer
    {
      // const std::lock_guard<std::mutex> lock(mtx_viewer);
      const std::lock_guard<std::mutex> lock(*mtx_viewer_ptr);
      pcl_viewer->spinOnce();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  driver.stop(); // stop when pcl viewer is closed

  // exit
  to_exit_process = true;
  cloud_handle_thread.join();

  return 0;
}