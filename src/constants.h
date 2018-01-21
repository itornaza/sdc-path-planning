#ifndef PATH_PLANNING_CONSTANTS_H_
#define PATH_PLANNING_CONSTANTS_H_

namespace Constants {

  // Track length
  const double MAX_S = 6945.554; // 4.32 miles
  
  const double MAX_V = 49.5;  // mph
  
  const double METRIC_2_MPH = 2.236936292;
  const double SPEED_INC = 0.22352; // in m/sec (i.e. 0.5 mph)
  
  // The simulator runs a cycle every 20 ms (50 frames per second)
  const double UPDATE_PERIOD = 0.02;
  
  const string MAP_FILE = "../data/highway_map.csv";
  
}

#endif /* PATH_PLANNING_CONSTANTS_H_ */
