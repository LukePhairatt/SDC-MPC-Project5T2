# **Model Predictive Control(MPC) Project-5 Term-2**
Self-Driving Car Engineer Nanodegree Program
![project][image0]
---

[//]: # (Image References)
[image0]: ./result/capture.png "result"
[image1]: ./result/track_waypoints.png "track"
[image2]: ./result/setpoint_velocity.png "speed"
[video0]: ./result/MPC.mp4 "video"


### The Control Model 
Note: the simulator steering CW is + and CCW is - (so the motion model is adjusted to this steering convention)

_Update equations for the states and errors:_
	
	x[t+1] 	    = x[t] + v[t] * cos(psi[t]) * dt			// x-position  
	y[t+1]      = y[t] + v[t] * sin(psi[t]) * dt 			// y-position  
	psi[t+1]    = psi[t] - v[t] / Lf * delta[t] * dt		// heading  
	v[t+1]      = v[t] + a[t] * dt					// speed  
	cte[t+1]    = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt		// cross-track error  
	epsi[t+1]   = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt	// orientation error

	where
		dt: a time step
		delta: a steering angle
		a: an acceleration/throttle 
		f(x): a predicted y position at x from a waypoint polynomial fitting 
		psides: a predict heading at x from a waypoint polynomial fitting 
		Lf: the vehicle steering length (estimated from a steering point to CG)
		
	Note:   for this project, the error terms are defined as follows 
		cte  = y_predict  - y_actual
		epsi = psi_actual - psi_predict

The optimal control outputs (steering and throttle) are computed by solving the update equations defined as above. This project uses **Ipopt** solver package for the optimisation of non-linear problem with the  MPC state variables (x, y, psi,v, cte, epsi, delta, a) and the following per-defined costs and constraints.

_Minimizing costs:_
The cost terms to be minimised in order to achieve the optimal solution (e.g. smooth, minimal errors) are as follows (MPC.cpp 37-55 lines):

* Cross-track, Orientation and Set-speed error 
* The use of actuators e.g. steering, throttle
* The rate of actuators

Further to the errors, there are a set of hyper-parameters to be tuned in the implementation for the good performance. These include 

Weight the error terms, this indicates how much we want to signify the errors.  
**etc_weight**   = 4.0;  
**epsi_weight**  = 2.0;  

Weight the actuator outputs, this indicates how much we want to penalise the outputs.  
**steering_penalise** = 25000.0;  
**throttle_penalise**  = 1.0;  
**steering_rate_penalise** = 50000.0;  
**throttle_rate_penalise**  = 2000.0;  

These numbers are derived from the experimentations.

_Model constraints (from the update equations):_ 
It is ideal to have these equations to be close to 0 where prediction of the next state is actually close to the actual state possible. 
	
	x[t+1] -  (x[t] + v[t] * cos(psi[t]) * dt)			= 0  
	y[t+1] -  (y[t] + v[t] * sin(psi[t]) * dt) 			= 0  
      	psi[t+1]  -  (psi[t] - v[t] / Lf * delta[t] * dt)		= 0  
      	v[t+1] - (v[t] + a[t] * dt)					= 0  
      	cte[t+1]  -  (f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt) 	= 0  
      	epsi[t+1] - (psi[t] - psides[t] - v[t] * delta[t] / Lf * dt) 	= 0  

All are set to 0, except the first initial point which is initialised to the initial state (MPC.cpp 131-138 lines). The boundary of the state variables, outputs and model constraints are defined in the MPC::Solve function (MPC.cpp  151-231 lines). In this project, the steering and throttle value are limited to -1 to +1. 

### Timestep Length and Elapsed Duration (N & dt)
Based on the experimentations, the value of N=15, and dt=0.05 gave the best result on this implementation. These parameters control how far ahead we want to look to the horizon. It has been observed that by looking too far ahead, the next corner (which is far ahead) might affect the vehicle path to turn too early. However, by looking too near a distance, this might not be enough for the vehicle to act upon the coming corner. For the experimentation, these sets of parameters were tried,
	
Too short look ahead (failed to turn)  
	N = 10 and dt = 0.01, 0.02, 0.03  
	N = 15 and dt = 0.01, 0.02, 0.03  
	
Too long look ahead (turned too quickly before the coming corner, and sometime it went off track)  
	N = 15 and dt = 0.08, 0.1   
	N = 20 and dt = 0.05, 0.08, 0.1  
	N = 25 and dt = 0.03, 0.05, 0.1  


### Polynomial Fitting and MPC Preprocessing
The given waypoints sent from the simulator are transformed to the vehicle frame before fitting with the polynomial function to ease computation of the update equations and stability of polynomial fitting as well as sending out the resulting waypoints and predicted points to the simulator. The transform equation is given by (main.cpp 172-178 lines):
	
	ptsx_v = (ptsx-px) * cos(psi) + (ptsy-py)*sin(psi)  
	ptsy_v = -(ptsx-px) *sin(psi) + (ptsy-py)*cos(psi)  

	where 	px,py,psi is the current vehicle pose  
		ptsx, ptsy is the given waypoints in the world coordinate  
		ptsx_v, ptsy_v is the given waypoints in the vehicle frame  


The reference path is then generated with the 3rd order polynomial function to suit this track on the twist and turn corners. 

	coeffs = polyfit(ptsx_v, ptsy_v, 3); 

The reference path is sent to the MPC solver by the polynomial coefficients along with the predicted vehicle state (due to 100 ms latency in the control outputs, see next section for details). 
	
_Reference speeds:_

![track][image1] _Track layout_
![speed][image2] _Set point speed_

The set point velocities on this track are predetermined from the track layout in the given waypoint log file (“lake_track_velocity.csv”). The reference the speed is then past to the MPC model for a variable speed control on the track. In the implementation, the log file is read into the x,y waypoint along with the expected velocity. During each motion step, the vehicle is localised to nearest waypoint to get the predefined target speed (see main.cpp 155-157 lines). The code for reading the log file is in track.cpp.

**main.cpp**
/* read track data- waypoints and predefined speeds*/
std::string filename = "../lake_track_velocity.csv";
ThrottleMapping lake_track; 
lake_track.ReadTrackData(filename);

/* get the vehicle current location closest to the waypoint */
int idx_map = lake_track.GetLocation(px,py);  
   
/* get the desire speed */     
double expect_velocity = lake_track.GetTrackVelocity(idx_map);

/* pass to mpc solver */
double target_speed = expect_velocity;
vector<double> mpc_solution = mpc.Solve(state, coeffs, target_speed);

### Model Predictive Control with Latency
In oder to deal with the control latency, the vehicle state is set to the next point based on the latency for the MPC solver. 

In the vehicle frame, at the current (initial) state: psi(heading)=0, x=0, y=0, v=v, a=a. The state update will only be in the x-direction, so no change in the y-position. 

The update equations in the vehicle frame are given by:  
	time_lag  = latency time  
	px_next   = v*time_lag;  
	py_next   = 0.0;  
	psi_next  = 0.0 - v*steering_angle*deg2rad(25)*time_lag/Lf;  
	v_next    =  v + a*time_lag;  
	cte_next  = polyeval(coeffs, px_next) - py_next;  
	epsi_next= psi_next - atan(coeffs[1] + 2*coeffs[2]*px_next + 3*coeffs[3]*pow(px_next, 2));   

NOTE: 	at any moment in time, we define the error terms with
	cte  = y_pred - y_actual
       	epsi = psi_actual - psi_pred

  	the steering angle needs to be rescaled from -1 to 1 radian (simulator limit) to the limited values as defined in the MPC variable boundary 		(-0.436 to 0.436 radian or -25 to 25 degree).  
	However, the steering from the MPC solution will be rescaled back to [-1, 1] before sending it to the simulator.    


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
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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
