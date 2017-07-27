# **Model Predictive Control(MPC) Project-5 Term-2**
Self-Driving Car Engineer Nanodegree Program
![project][image0]
---

[//]: # (Image References)
[image0]: ./result/capture.png "result"
[image1]: ./result/track_waypoints.png "track"
[image2]: ./result/setpoint_velocity.png "speed"
[video0]: ./result/MPC2.mp4 "video"

The project was tested on this simulator configuration.

Version: 145  
Graphic: 1024 x 768  
Quality: fast, simple, good, fantastic  
Estimated message time: ~ 0.1-0.2 second  

However, it should works on any configuration and machines, provided the I/O message is not severely delayed.


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
**etc_weight**   = 2000.0;  
**epsi_weight**  = 1800.0;  

Weight the actuator outputs, this indicates how much we want to penalise the outputs.  
**steering_penalise** = 20.0;  
**throttle_penalise**  = 10.0;  
**steering_rate_penalise** = 400.0;  
**throttle_rate_penalise**  = 20.0;  

These numbers are derived from the experimentations.

_Model constraints (from the update equations):_

It is ideal to have these equations to be close to 0 where prediction of the next state is actually as close as to the actual state possible.  

	x[t+1] -  (x[t] + v[t] * cos(psi[t]) * dt)			= 0  
	y[t+1] -  (y[t] + v[t] * sin(psi[t]) * dt) 			= 0  
	psi[t+1]  -  (psi[t] - v[t] / Lf * delta[t] * dt)		= 0  
	v[t+1] - (v[t] + a[t] * dt)					= 0  
	cte[t+1]  -  (f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt) 	= 0  
	epsi[t+1] - (psi[t] - psides[t] - v[t] * delta[t] / Lf * dt) 	= 0  

All are set to 0, except the first initial point which is initialised to the initial state (MPC.cpp 131-138 lines). The boundary of the state variables, outputs and model constraints are defined in the MPC::Solve function (MPC.cpp  151-231 lines). In this project, the steering and throttle value are limited to -1 to +1. 

### Timestep Length and Elapsed Duration (N & dt)
Based on the experimentations, the value of N=10, and dt=0.1 gave the best result on this implementation. These parameters control how far ahead we want to look to the horizon. It has been observed that by looking too far ahead, the next corner (which is far ahead) might affect the vehicle path to turn too early. In addition, this adds overhead to the solver. In contrast, by looking too near a distance, this might not be enough for the vehicle to act upon the coming corner. For the experimentation, these sets of parameters were tried,
	
Too short look ahead (failed to turn)  
	N = 8  and dt = 0.05, 0.1  
	N = 9  and dt = 0.05, 0.1

Too long look ahead (turned too quickly before the coming corner, and sometime it went off track)  
	N = 15 and dt = 0.1   
	N = 20 and dt = 0.05, 0.08, 0.1  
	N = 25 and dt = 0.03, 0.05, 0.1  


### Polynomial Fitting and MPC Preprocessing
The given waypoints sent from the simulator are transformed to the vehicle frame before fitting with the polynomial function to ease computation of the update equations and stability of polynomial fitting as well as sending out the resulting waypoints and predicted points to the simulator. The transform equation is given by (main.cpp 172-178 lines):
	
	ptsx_v = (ptsx-px) * cos(psi) + (ptsy-py)*sin(psi)  
	ptsy_v = -(ptsx-px) *sin(psi) + (ptsy-py)*cos(psi)  

	where 	px,py,psi is the current vehicle pose  
		ptsx, ptsy is the given waypoints in the world coordinate  
		ptsx_v, ptsy_v is the given waypoints in the vehicle frame  


The reference path is then generated using the 3rd order polynomial function to suit this track with the twist and turn corners. 

	coeffs = polyfit(ptsx_v, ptsy_v, 3); 

The reference path is sent to the MPC solver by the polynomial coefficients along with the predicted vehicle state (due to 100 ms latency in the control outputs, see next section for details). 
	
_Reference speeds:_

The target speed for the MPC could be set to the constant maximum boundary for this project. The MPC solution will find the speed to  
minimise the overall cost functions. However, the approch which is taken here explores the use of variable speed control as follows.  

![track][image1]   
                              _Track layout_

![speed][image2]   
                              _Set point speed_

**The is optional. This MPC project works without the speed mapping.** (However, I believe it makes the vehicle react in the more robust and precise manner).

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
	psi_next  = 0.0 - v*steering_angle*time_lag/Lf;  
	v_next    =  v + a*time_lag;  
	cte_next  = polyeval(coeffs, px_next) - py_next;  
	epsi_next= psi_next - atan(coeffs[1] + 2*coeffs[2]*px_next + 3*coeffs[3]*pow(px_next, 2));  

	NOTE: 	
	at any moment in time, we define the error terms with  
	cte  = y_pred - y_actual  
	epsi = psi_actual - psi_pred  

  	the steering from the MPC solution will be rescaled to [-1, 1] before sending it to the simulator.    


### result
![video][video0]


