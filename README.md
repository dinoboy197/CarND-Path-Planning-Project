# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The input provided is the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---

## Video of vehicle control

Please see a video of the path planner I implemented in this project controlling the vehicle in the Term 3 simulator [here](https://youtu.be/oyjX6oERRVc).

## Implementation details

The main implementation pieces of this project include:

* Other vehicle tracking
* Lane change decision making
* Speed control
* Trajectory generation

### Other vehicle tracking

The [vehicle tracking](src/main.cpp#L369-L417) section of code is responsible for using data from sensor fusion measurements to determine where other vehicles are relative to the controlled vehicle. This includes detecting vehicles directly to the left, ahead, and right and determining how much empty space exists between these vehicles. This information is used for the lane change decision making system.

### Lane change decision making

The [lane change decision making](src/main.cpp#L419-L432) system determines if the vehicle should attempt to change lanes if it is currently travellling at less than 90% of the maximum speed (50 MPH) and the vehicle is following less than 50 units behind the vehicle in front of it. Once that determination is made, the vehicle looks to both the left and right for an opening with no other vehicles. If both lanes are open, the lane with a vehicle farthest ahead is chosen for the lane switch.

### Speed control

The [speed control](src/main.cpp#L434-L439) system maximizes the speed in the current lane given traffic conditions. If no other vehicles are within 50 units ahead, the system accelerates slowly until almost 50 MPH is reached. Otherwise, the vehicle slows down to match the ahead vehicle's speed, and maintains a distance from the ahead vehicle equal to the current velocity (a proportional following distance).

To achieve a constant following distance, a [PID controller](src/PID.cpp) is used with proportional and differential controls turned on.

### Trajectory generation

Once a target lane and velocity are computed, a trajectory is calculated several steps into the future ([code 1](src/main.cpp#L441-L448), [code 2](src/main.cpp#L171-L269)). [On a first trajectory generation run, the vehicle's location and previous point using the vehicle's yaw angle are computed; otherwise,  the number of points remaining in the trajectory are obtained](src/main.cpp#L186-L210). Next, [three points evenly spaced down the target lane](src/main.cpp#L212-L215) are generated. These points are converted to x-y coordinates, and they are [angle-corrected](src/main.cpp#L226-L232) for the vehicle's origentation in the environment. [Spline interpolation](src/main.cpp#L235-L241) is used for [computing additional trajectory points](src/main.cpp#L235-L241), which when combined with any previous trajectory points from a previous iteration not traveled, to [bring the total number of points up to 50](src/main.cpp#L250-L268). These points are then handed off to the simulator for the vehicle to follow.

## Results

Using these components with input data from the simulator and existing map data (described below), this path planner is able to drive the simulated vehicle safely and efficiently around the course, maximizing its speed while avoiding collisions. Simulation runs are often approach hundreds of miles before any kind of collision is experienced, and multiple runs of the simulator show this very often due to unsafe and sudden behavior from other vehicles.

***

## Input Data

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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


---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Dependencies

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

