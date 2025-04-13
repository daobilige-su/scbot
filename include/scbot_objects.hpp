#ifndef SCBOT_OBJECTS_HPP_
#define SCBOT_OBJECTS_HPP_

namespace scbot { // namespace for solar panel clearer robot

struct ObstaclePanel {
  // general params
  bool isPanelExist = false;
  float dis2panel = 10.0F;

  // gap params
  bool isGapExist = false; // gap within 5m along X dir
  float dis2gap = 0.0F;

  // panel plane params
  float Panel_A = 0.0F;
  float Panel_B = 0.0F;
  float Panel_C = 0.0F;
  float Panel_D = 0.0F;

  // top line: its anchor point and direction
  float LINE1_x = 0.0F;
  float LINE1_y = 0.0F;
  float LINE1_z = 0.0F;
  float LINE1_m = 0.0F;
  float LINE1_n = 0.0F;
  float LINE1_p = 0.0F;

  // bot line: its anchor point and direction
  float LINE2_x = 0.0F;
  float LINE2_y = 0.0F;
  float LINE2_z = 0.0F;
  float LINE2_m = 0.0F;
  float LINE2_n = 0.0F;
  float LINE2_p = 0.0F;
};

enum ObstacleType : int {
  OBJ_T_UNKNOWN = 0,
  OBJ_T_RADAR_UNKNOWN = 1,
  OBJ_T_PEDESTRIAN = 2,
  OBJ_T_TRUCK = 3,
  OBJ_T_CAR = 4,
  OBJ_T_TRACTOR = 5,
  OBJ_T_SHEEP = 6,
  OBJ_T_CATTLE = 7,
  OBJ_T_HILL = 8,

  AREA_T_BLINDZONE = 28
};

struct ObstacleObj {
  bool isObjValid = false;
  int ID = 0;
  ObstacleType type = ObstacleType::OBJ_T_UNKNOWN;
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double speed_x = 0.0;
  double speed_y = 0.0;
  double speed_z = 0.0;
  double heading = 0.0;
  double box_length = 1.0;
  double box_width = 1.0;
  double box_height = 1.0;
  float confidence = 0.75F;
  double time_stamp = 0.0;
};

} // namespace scbot

#endif