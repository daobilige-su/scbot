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

} // namespace scbot

#endif