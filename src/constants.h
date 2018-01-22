#ifndef PATH_PLANNING_CONSTANTS_H_
#define PATH_PLANNING_CONSTANTS_H_

namespace Constants {

  const string MAP_FILE = "../data/highway_map.csv";
  const double MAX_S = 6945.554; // 4.32 miles track length
  const double MAX_V = 49.5;  // mph
  const double SPEED_INC = 0.22352; // in m/sec (i.e. 0.5 mph)
  const double METRIC_2_MPH = 2.236936292;
  
  // The simulator runs a cycle every 20 ms (50 frames per second)
  const double UPDATE_PERIOD = 0.02;
  
  // Behavior planner
  const int FOLLOW_LEAD_S = 30;
  const int MIN_FOLLOW_LEAD_S = 20;
  const int TRAILER_S = 30;
  
  // Spline
  const int NEAR_POINT = 30;
  const int MID_POINT = 60;
  const int FAR_POINT = 90;
}

#endif /* PATH_PLANNING_CONSTANTS_H_ */
