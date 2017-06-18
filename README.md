# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
# Project Write Up

## Introduction
---
This project implements a model predictive control, MPC, to have a vehicle drive around in a simulator.

## Requirements
---
### The Model
* Student describes their model in detail. This includes the state, actuators and update equations.
### Timestep Length and Elapsed Duration (N & dt)
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
### Polynomial Fitting and MPC Preprocessing
* A polynomial is fitted to waypoints
* If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
### Model Predictive Control with Latency
* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
### Simulation: Parameter Tuning
* No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

## Discussion
---
### The Model
The model consists of 6 state variables that set in main.cpp based on simulation data:
* *x*: the x position of the vehicle
* *y*: the y position of the vehicle
* *psi*: the yaw of the viehicle in radians
* *v*: the velocity of the vehicle
* *cte*: the cross track error of the vehicle given its current state and defined course from the simulator
* *epsi*: the error in the yaw of the vehicle given its current state and defined course from the simulator

The vehicle has two actuators:
* *a*: the throttle/accelaration
* *delta*(*6*): the steering angle in radians

The class FG_eval in MPC.cpp implements following state update equations:
* *x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos(ѱ<sub>t</sub>) dt*
* *y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin(ѱ<sub>t</sub>) dt*
* *ѱ<sub>t+1</sub> = ѱ<sub>t</sub> + v<sub>t</sub> δ dt / L<sub>f</sub>*, where
*L<sub>f</sub>* is a physical parameter of the vehicle
* *v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> dt*
* *dt*, the change in time is defined in MPC.cpp
The update equations allow the MPC to consider the future position of the vehicle given a set of commands at each time step. The MPC uses a cost function to minimize the error of the cost components, the update equations, actuator values and the waypath the vehicle should travel to optimize the actuator values at each measurement step given future positions. This makes the MPC dynamic since it considers the future waypath of the vehicle when optimizing actuator commands right now.

Model Predictive Control (MPC) uses an optimizer to find the control inputs that minimize the cost function and follows the following algorithm:
Here is the MPC algorithm:

#### Setup:

* Define the length of the trajectory, *N*, and duration of each timestep, *dt*
** values set in MPC.cpp
* Define vehicle dynamics and actuator limitations along with other constraints
** constants defined at the top of MPC.cpp and set in class function MPC::Solve
* Define the cost function
** defined in class FG_eval with a cost component for *cte*, *epsi*, *v*, *a*, *6*, *da/dt* and *d6/dt*
** each cost component has a weight parameter to tune which component has a higher weight when optimizing

#### Loop:

* pass the current state as the initial state to the model predictive controller
** values received from simulator and processed into car's reference frame from global reference system for each point on desired path for the vehicle (waypoints)
** fit polynomial to the desire path
* call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver used is called Ipopt.
** this passed the current state vector and fitted coefficients for the vehicle path to MPC:Solve
* apply the first control input to the vehicle.
** MPC::solve returns the first accuator values for the throttle and steering angle
** the fitted trajectory from the MPC::Solve is passed to the simulator as well as the desired path
* back to pass the current state as the initial state to the model predictive controller

### Timestep Length and Elapsed Duration (N & dt)

Ths initial values that I tried for the number of time steps and the change in time was 10 and 0,1 respectively. I tuned these parameters along with the tuning constants for the cost function. I found that 10 time steps at 0.1 wasn't plotting a very long trajectory path of the vehicle and I thought this may be causing some of the issues with overcorrecting the steering angle. I reduced the time step to 0.05 and increased the number of steps to 30. This when I had the cost function parameters for the throttle and steering angle set 1000.0 to see if would stabilize the path. It did not.
I left time step to 0.05 and the number of steps to 30 while I tuned the other parameters and ended up getting the vehicle to drive around the track without having to retune these. I did see that it was projecting far enough ahead of the vehicle that it was taking into account up coming turns and following the fitted path quite nicely. Those are the reasons I chose to not change the number of time steps or the change in time between time steps.

### Polynomial Fitting and MPC Preprocessing

Before I fit the simulator waypoints to a 3rd order polynomial I convert the waypoints to the car's reference frame using translation and ingoring rotation. These transformed waypoints are fitted to a 3rd order polynomial. The reason I chose the car's reference frame is the path the vehicle needs to follow is always going to being in reference to its current state at point (0,0) since the origin is at the car. Converting to this reference frame right at the beginning makes solving this problem much easier. Once this was done the fitted waypoint polynomail first coefficient is the *cte* of the car and the negative arctan of the second coefficient is *epsi*. With this the state vector is 0,0,0,*v*,*cte*,*epsi* since the car is the origin in its reference frame and rotation was ignored so the *yaw* is 0.

### Model Predictive Control with Latency

Latency is dealt with by capping the maximum speed to 30
I also dealt with it by not putting much weight on the velocity error in the cost function
The vehicle never actually reaches the maximum speed but that's because the weight of the 
cost function is place on the value steering angle and throttle as well as the rate of change of the steering angle and throttle over time.
If either of these two assumptions were broken the model would need to take latency into account as the velocity increases.
Latency could be dealt with by choosing *delta*(*6*) and *a* values that are at a future time step from the MPC::Solve function. The current setting is to choose the first optimized *delta*(*6*) and *a* values but it could be set to choose a time step in the future to account for latency. This would be needed for higher reference speeds since more distance is travelled at each *dt* step.

### Simulation: Parameter Tuning
Below is how I tuned the cost function weight parameters.
These paramters were tuned with a target velocity of 30.
I started with all of these tuning parameters for the cost function at 1.0. I started with the rate of change of the actuator values, TUNE_DELTA_SEQ_DIFF and TUNE_ACC_SEQ_DIFF, since I wanted to start by smoothing the rate of change of the actuators. The vehicle was all over the road and the path was shooting left and right with all the cost parameters at 1.0.

After that I set TUNE_DELTA_SEQ_DIFF to 1000.0 and compiled to code to find the change vehicle trajectory stabilized but the throttle was go and stop, go and stop so I then set TUNE_ACC_SEQ_DIFF to 1000.0. This help stabilize the change to the throttle over time. That being said the program still had initial large steering angles and throttle.

I set TUNE_ACC_VAL to 1000.0 to prevent large changes in values for the throttle. This value was far too large and the car was moving at a snails pace. I set this value to 100.0 and left it there.

I then set TUNE_DELTA_VAL to 1000.0 and this prevented the steering angle from reaching values large enough to take curves. I lowered it to 200.0 but found the vehicles path to be unstable with it making over corrective turns. I then attempted settings it to 350.0 and found this was giving ideal driving behavior.

Once I had the cost parameters tuned to the below settings the vehicle drove around the track. The finalized values are:
* const double TUNE_CTE = 1.0;
* const double TUNE_EPSI = 1.0;
* const double TUNE_VEL = 1.0;
* const double TUNE_DELTA_VAL = 350.0;
* const double TUNE_ACC_VAL = 100.0;
* const double TUNE_DELTA_SEQ_DIFF = 1000.0;
* const double TUNE_ACC_SEQ_DIFF = 1000.0;

# Project Setup
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
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
