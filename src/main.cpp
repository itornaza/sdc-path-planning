#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"
#include "constants.h"

using namespace std;
using namespace Helpers;
using namespace Constants;

// For convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  ifstream in_map_(MAP_FILE.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1
  int lane = 1;
  int lane_width = 4;
  int lane_change_wp = 0;
  
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane, &lane_width,
               &lane_change_wp]
               (uWS::WebSocket<uWS::SERVER> ws,
                char *data, size_t length,
                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
        	
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's end (s, d) values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Get a list of all other cars on the same side of the road from the
          // Sensor Fusion block
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Holds calculated variables to send to the simulator in json format
          json msgJson;
          
          //********************************************************************
          // Localization
          //********************************************************************
          
          double ref_vel = MAX_V;
          int prev_size = previous_path_x.size();
          int next_wp = -1;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2) {
            next_wp = NextWaypoint(ref_x, ref_y, ref_yaw,
                                   map_waypoints_x, map_waypoints_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            ref_y = previous_path_y[prev_size - 1];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            car_s = end_path_s;
            next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x,
                                   map_waypoints_y);
            car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) +
                              (ref_y - ref_y_prev) * (ref_y - ref_y_prev)
                              ) / UPDATE_PERIOD) * METRIC_2_MPH;
          }
          
          //********************************************************************
          // Behavior planner and sensor fusion
          //********************************************************************
          
          double closestDist_s = MAX_S;
          bool change_lanes = false;
          bool check_right = true;
          
          // Car is in my lane
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            
            if (d > get_lane_min_d(lane, lane_width) &&
                d < get_lane_max_d(lane, lane_width)) {
              
              // Get target car state
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt((vx * vx) + (vy * vy));
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size * UPDATE_PERIOD * check_speed);
              
              // Check s values greater than mine and s gap
              if((check_car_s > car_s) &&
                 ((check_car_s - car_s) < 30) &&
                 ((check_car_s - car_s) < closestDist_s)) {
               
                closestDist_s = check_car_s - car_s;
                if(closestDist_s > 20) {
                  // Match that front car speed
                  ref_vel = check_speed * METRIC_2_MPH;
                  change_lanes = true;
                } else {
                  // Go slightly slower than the front car speed
                  ref_vel = check_speed * METRIC_2_MPH - 5;
                  change_lanes = true;
                }
              }
            } // End if - Car is in my lane
          } // End for - Sensor fusion
          
          if (change_lanes) {
            // Check to the left
            if (lane - 1 >= 0) {
              for (int i = 0; i < sensor_fusion.size(); i++) {
                // Mark if there is any other car +/- 30m on the left lane
                float d = sensor_fusion[i][6];
                if (d > get_lane_min_d(lane - 1, lane_width) &&
                    d < get_lane_max_d(lane - 1, lane_width)) {
                  
                  // Get target car state
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt((vx * vx) + (vy * vy));
                  double check_car_s = sensor_fusion[i][5];
                  
                  // Check gap
                  if (!(car_s - check_car_s < 30 && car_s - check_car_s > -30)) {
                    lane -= 1;
                    check_right = false;
                  }
                }
              }
            }
            
            // Check to the right
            if (check_right && (lane + 1 <= 2)) {
              for (int i = 0; i < sensor_fusion.size(); i++) {
                // Mark if there is any other car +/- 30m on the left lane
                float d = sensor_fusion[i][6];
                if (d > get_lane_min_d(lane + 1, lane_width) &&
                    d < get_lane_max_d(lane + 1, lane_width)) {
                  
                  // Get target car state
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt((vx * vx) + (vy * vy));
                  double check_car_s = sensor_fusion[i][5];
                  
                  // Check gap
                  if (!(car_s - check_car_s < 20 && car_s - check_car_s > -10)) {
                    lane += 1;
                  }
                }
              }
            }
          }
          
          //********************************************************************
          // Trajectory generation using spline
          //********************************************************************
          
          // Define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          
          // Create a list of widely spaced (x, y) points
          vector<double> ptsx;
          vector<double> ptsy;
          
          if(prev_size < 2) {
            // If previous size is almost empty use car as starting reference
            // Use 2 points that make a path tangential to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Use the previous path end point as starting reference
            // Redefine reference state as previous path end point and use these
            // 2 points that make a path tangential to the previous path end
            // point
            ptsx.push_back(previous_path_x[prev_size - 2]);
            ptsx.push_back(previous_path_x[prev_size - 1]);
            ptsy.push_back(previous_path_y[prev_size - 2]);
            ptsy.push_back(previous_path_y[prev_size - 1]);
          }
          
          // In Frenet coordinates add 30m spaced points ahead of the starting
          // reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane),
                                          map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane),
                                          map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane),
                                          map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++ ) {
            // Shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          // Create a spline
          tk::spline s;
          
          // Get the (x, y) points to the spline
          s.set_points(ptsx, ptsy);
          
          // Vectors to hold the generated trajectory (x, y) points
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all the previous path points from the last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we travel at our
          // desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) +
                                    (target_y) * (target_y));
          
          // Starting at the origin
          double x_add_on = 0;
          
          // Fill out the rest of our path planner, after filling it with
          // previous points. Always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            if (ref_vel > car_speed) {
              car_speed += SPEED_INC;
            } else if (ref_vel < car_speed) {
              car_speed -= SPEED_INC;
            }
            
            double N = (target_dist /
                        (UPDATE_PERIOD * car_speed / METRIC_2_MPH));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to normal (after rotating it earlier)
            x_point = (x_ref *cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref *sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            // Accumulate the points of the path
            x_point += ref_x;
            y_point += ref_y;

            // Push the points into the final trajectory
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          //********************************************************************
          // - End project code
          //********************************************************************
          
          // Send trajectory to the simulator for execution
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        } // End if - telemetry
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      } // End if/else  - autonomous/manual driving
    } // End outer if
  });

  /**
   * We don't need this since we're not using HTTP but if it's removed the
   * program doesn't compile :-(
   */
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
