# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, and pass slower traffic when possible, while other cars try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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
### Simulator Setup

TThe instructions are from [here](https://medium.com/@kaigo/how-to-install-udacitys-self-driving-car-simulator-on-ubuntu-20-04-14331806d6dd).

1. Download the (.deb) package of Unity (3D) version 5.5.1f1 that the Udacity Simulator uses. 
2. Install dependencies : `sudo apt install gconf-service lib32gcc1 lib32stdc++6 libc6-i386 libgconf-2-4 npm`
3. Run the install : `sudo dpkg -i ~/Downloads/unity-editor_amd64-5.5.1xf1Linux.deb`
4. If you get error about unmet dependencies you may need to run, and retry ` sudo apt --fix-broken install` 
5. Download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

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

## Rubric

### Compilation

The code compiles correctly. I did not change the cmake file. The only file that I added was [src/spline.h](https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/src/spline.h) for the Cubic Spline Interpolation [single header file](https://kluge.in-chemnitz.de/opensource/spline/) as shown in the Q&A video.

### Valid Trajectories

I performed several simulations to ensure that the car drove safely.

1. The car is able to drive for a distance greater than 4.32 miles without incident.

|Sim 1|Sim 2|Sim 3|
|:---:|:---:|:---:|
|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/4p6miles_1.jpg" width="480" height="270"/>|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/4p6miles_2.jpg" width="480" height="270"/>|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/4p9miles_3.jpg" width="480" height="270"/>|

2. The car drives at or below the speed limit without unnecessarily slowing down.
3. The car does not exceed total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
4. The car does not collide with other road users.

|Sim 4|Sim 5|Sim 5|
|:---:|:---:|:---:|
|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/8p8_miles.jpg" width="480" height="270"/>|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/17p66_miles.jpg" width="480" height="270"/>|<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/21p75_miles.jpg" width="480" height="270"/>|

I let the fifth simulation above run for over twenty miles or half hour and there were no incidents observed

5. The car stays in the lane except while changing lanes. It does not take too long to change lanes and is always in the right three lanes.
6. The car is able to perform smooth lane changes safely.
<img src="https://github.com/prasadshingne/CarND-Path-Planning-Project/blob/master/output_images/lane_change.jpg" width="480" height="270"/>

### Reflection

The Q&A video was very helpful and following it got me nearly to the finish.

The code is divided into three parts -

#### Sensor fusion, safety check and rule based behavior 
Lines 123 to 156 include code to use sensor fusion in order to find the open lanes based on a safety check. First, go through the senesor fusion list and find the lane in which the next car is. Then find the total speed and Frenet distance of that car. Then if this car is in th ego lane check if it 30 m ahead of the ego car. If the next car is in the left or right lane then check if it is greater then +/-30 m from the ego car.

Lines 159 to 174 specify code for the rule based behavior of the car based on the open lanes. If the next car is in the ego lane and ego car is within 30 m behind it check if either the left or right lanes are marked as safe. If either of the other lanes are safe change to that lane. If neither of the lanes are safe then reduce speed by 0.4 m/s (~0.9 mph). If the ego car is more than 30 m behind the next car in the ego lane then increase speed up to the speed limit by 0.22352 m/s (= 0.5 mph).

#### Trajectory generation
Lines 181 to 288 define the code for the vehicle trajectory. Line 184 to 219 gather the reference state of the car from previous path points. Then three (far) points along the trajectory ahead of the reference at 30 m, 60 m and 90 m are added to initialize the spline. Before using the spline the points are transformed to the ego car coordinates (lines 222 to 242). To ensure continuity the previous path points are added to the new trajectory (line 244 to 258). The rest of the points are added by calculating the spline and transforming back to map reference frame (line 261 to 286). 

Note: I have commented the code but it can be further cleaned up. Further, we can create an optimal trajectory by using a finite state machine and cost function which I haven't done. I would like to do this in the near future.







