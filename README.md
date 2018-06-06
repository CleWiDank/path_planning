# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Model Documentation:
With this documentation I would like to describe my code for the path planning project.
In a first step I took the input from sensor fusion and the ego car’s localization data to
evaluate some properties of the ego car and further cars, so called co-cars. To locate cocars
relative to the ego car, I thought the lane and the distance in the driving direction
should be of largest relevance. With the calculated speed (m/s), s-position (in driving
direction) and lane (0, 1, 2) I define that a car should be flagged as car_ahead if it is
closer than 30m in front of the ego car. Furthermore I flag a car as car_left or car_right, if
it is in the range of 30m in front of or behind the ego car. The rationale behind this is,
that a lane change might be dangerous if a car is within this range.

Those evaluations are taken into account for behavior planning. The car is smoothly
accelerating with 5m/s (0.224 mph / 20ms) up to the speed limit of nearly 50mph. It
keeps this maximum legal speed until it finds a car that is flagged as car_ahead. In this
case it determines if it can undertake a lane change without slowing down. Therefore, it
checks the adjacent lanes 30m in front and behind itself. If those are not free, the ego car
slows down with -2.5m/s until its own speed equals the speed of the car ahead. Having
reached it the ego car follows the speed of the car ahead (Active Cruise Control) while
keeping a safe distance. In the meantime it always checks if a safe lane change becomes
possible in order to move faster. When the car ahead is gone (either because it
accelerated or the ego undertook a lane change) the ego car accelerates smoothly to the
speed limit again.

The ego car follows the described logic by visiting calculated points that lie in front of
the car. Every 20ms the car visits the next point so the distance between the points
determines the speed. Three target points that are equally spaced points in the future
are taken to build up a spline that works well for smooth lane changes without too much
jerk. Meanwhile, the points close by the car are taken as starting reference for the
heading direction. The points of the previous path that are not visited by the car are
reused within the next frame. To fill up the point lane I generate “add on” points which
are equally spaced compared to the existing point lane and in accordance to the spline.
Comment: To run the code you need to add the Eigen-library to src.  

---

## Setting up the enviroment:
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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
    ``
