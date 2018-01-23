# Path Planning

## Introduction

The planning path program safely navigates around a virtual highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit.

The car's localization and sensor fusion data are provided by the simulator, there is also a sparse map list of waypoints around the highway. The car drives as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, while other cars will try to change lanes too. The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. Also the car does not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The project rubric can be found [here](https://review.udacity.com/#!/projects/318/rubric).

![alt text](https://github.com/itornaza/sdc-path-planning/blob/master/images/cover.png "Path planning in action")
![alt text](https://github.com/itornaza/sdc-path-planning/blob/master/images/cover_2.png "Path planning in action")

A video that shows the self driving car driving down the highway on [youtube](https://youtu.be/xbWaVIDDG8A).

## Behavior planner

In order for the vehicle to safely navigate around the track and be able to pass slower vehicles in order for it to reach the end of the track in an acceptable timeframe it needs to adhere a set of rules. These rules define how it tackles situations that there is slower traffic ahead that blocks its way.

The code that defines this behavior can be found in the `/src/main.cpp` and from `line 132` to `line 246`. The car is driving close to the speed limit until another slower car blocks the lane that our car is driving. When this happens, our car progressively slows down. The follow traffic block can be found in  `/src/main.cpp` and from `line 145` to `line 176`.

As this happens, the car tries to find a passing window first on the left, and if that fails, on the right of the leading traffic. If it finds a clear window, it initiates a passing maneouver and accelerates towards the speed limit once again. In order to locate a clear window, it searches for other vehicles in the lane of interest (first left and then right) 32m ahead of it and 25m behind it. If no other car blocks its way, it goes ahead and passes the slow traffic. For more details on the implementation look at `/src/main.cpp` and from `line 184` to `line 245`.

The information about the other traffic is provided by the sensor fusion block from the simulator in json format. The information for each car is captured and adjusted for the simulator latency in order to perform safe predictions about its position and velocity. See `lines 150-156`.

All the calculations for the relative location between our car and the traffic is done in the Frenet domain using the `s` and `d` coordinates for the longitudinal and lateral distance between the vehicles. Where `s` provides us the window in the longitudinal direction in order for our car to decide if there is enough room to pass the slow vehicle, and `d` is used to describe the lanes of interest.

## Trajectory generation

The trajectories are generated using [slpine](http://kluge.in-chemnitz.de/opensource/spline/). The spline function is in a single hearder file is really easy to use instead of polynomials of the fifth order for the needs of this project.

The generation of the trajectories using the spline tool is described in detail in the "Project Walkthrough and Q&A" [video](https://www.youtube.com/watch?v=7sI3VHFPP0w).

The trajectory is defined by a path of (x,y) points that are visited sequentialy every 0.02 seconds from the simulator. In order to create a smooth trajectory, we use the last 2 points of the previously calculated trajectory provided to us by the simulator. Having these two points, we calculate the tanential angle as the direction of the car and start from that.

We then create three points at 30m, 60m, and 90m ahead of the car as anchor points for the spline tool. We shift the coordinates from vertical to horizontal to avoid errors that are generated close to the vertical direction. After doing so, we create the spline and evenly space the 50 points that we need within the curve. As a final step, we shift back the coordinates to the original vertical setting and push the trajectory to the simulator.

The trajectory generation block can be found in the `/src/main.cpp` and from `line 252` to `line 367`.

## Notes

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. The map of the highway is in `data/highway_map.txt`. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Data provided by the simulator

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Basic Build Instructions

### Simulator

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

### Run the program

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

