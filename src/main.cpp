#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "MPC.h"
#include "track.h"

// for convenience
using json = nlohmann::json;
using CppAD::AD;

// [x,y,psi,v,cte,epsi]
Eigen::VectorXd state(6); 
std::vector<double> old_outputs = {0.0,0.0};
int loop_count = 0;
// adding delay to the controller
const int milli_latency = 100;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Compute track curvature
double TrackCurvature(Eigen::VectorXd xvals, Eigen::VectorXd yvals)
{
  /*
    finding 3rd order polyfit curve using inverse for better stability
    in case of a vertical line (car move forward so we won't worry much about horizontal line) 
    x = f(y) = Ay**3 + By**2 + Cy + D
    A = coef[3], B = coef[2], C = coef[1], D = coef[0]
    x'(y)  = 3Ay**2 + 2By + C
    x''(y) = 6Ay + 2B 
    curvature equation R =  ( 1+ (x')**2)**1.5     evaluated at any y
                           ---------------------
                                   abs(x'')
  */
  
  //using 2nd order
  auto coef = polyfit(yvals, xvals, 2);          // given y get x_pred x = f(y)
  double A = coef[2];
  double B = coef[1];
  // average radius 
  int n = 0;
  double R = 0.0;
  for(int i= 0;i<yvals.size(); i++)
  {
	double y_val = yvals[i];   
    double x_dot = 2.0*A*y_val + 2.0*B;
    double x_dotdot = 2.0*A;
    R += (CppAD::pow( (1.0 + x_dot*x_dot), 1.5))/fabs(x_dotdot);
    n++; 
  }
  R /=n;
  
  return R;
}



int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  
  // Track speed mapping and Waypoints generator 	 
  //std::string filename = "../lake_track_waypoints_velocity.csv";
  std::string filename = "../lake_track_velocity.csv";
  ThrottleMapping lake_track; 
  lake_track.ReadTrackData(filename);
  
  h.onMessage([&mpc, &lake_track](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"]; // Note simulator use different direction (CW+,CCW-)
          double v   = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
                    
          /*
             Get waypoint velocity from the current location
          */
          int idx_map = lake_track.GetLocation(px,py);          
          double expect_velocity = lake_track.GetTrackVelocity(idx_map);
          std::cout << "Set speed = " << expect_velocity << std::endl;		 
		  
          /*
            Compute optimal control in the vehicle frame
            Given waypoints are in world frame so transform them to the vehicle frame, also
            Simulator needs reference points in this vehicle frame to display step by step 
          
                  ptsx = x * cos(psi) - y* sin(psi) + px    ---1
                  ptsy = x * sin(psi) + y* cos(psi) + py    ---2
             
             solving for x,y (pts points in vehicle frame)        
             1xcos,2xsin and 1xsin, 2x(-cos) then we have
                  x = (ptsx-px) * cos(psi) + (ptsy-py)*sin(psi)
                  y = -(ptsx-px) *sin(psi) + (ptsy-py)*cos(psi)             
          */ 
          Eigen::VectorXd ptsx_v(ptsx.size());
          Eigen::VectorXd ptsy_v(ptsy.size());
          for(int i=0; i<ptsx.size(); i++)
          {
		    ptsx_v[i] =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
		    ptsy_v[i] = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
		  }
		 
		  //Track curvature
		  //Eigen::VectorXd WPx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
		  //Eigen::VectorXd WPy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());
		  //double Curvature = TrackCurvature(ptsx_v,ptsy_v);
		  //double Curvature = TrackCurvature(WPx,WPy);
		  //std::cout << "Coming curvature = " << Curvature << std::endl;
		 
		 
		  /*
               MPC Solution- Calculate steering angle and throttle
               Both are in between [-1, 1].
          */	
		 
		  // Road is curvy so a polynomial fit with order 3
          auto coeffs = polyfit(ptsx_v, ptsy_v, 3);
          
          /* NOTE:
             Vehicle kinematic model: steering CW(+),steering CCW(-) expected by the simulator
             x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
             y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
             psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt
             v_[t+1] = v[t] + a[t] * dt
             cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
             epsi[t+1] = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt
          
             NOTE: at any moment in time, we difne these in mpc
             cte  = y_pred - y_actual                   ----(A)
             epsi = psi_actual - psi_pred               ----(B)
          */
          
          // Add latency to the state and errors before finding the optimised controls in  the VEHICLE frame
          // where at the current state t: psi(heading)is now 0, x=0, y=0, v=v
          double time_lag  = (double)milli_latency/1000.0;              // secnd next time step
          double at        = old_outputs[1]*time_lag;                   // estimate current throttle from the previous loop
          // predicte next state what will be
          double px_next   = v*time_lag;                                // only move in x vehicle frame
          double py_next   = 0.0;                                       // no change in y so 0.0 as y initial
          double psi_next  = -v*steering_angle*deg2rad(25)*time_lag/Lf; // rescale steering from [-1,1] radian to 25 degree radian
          double cte_next  = polyeval(coeffs, px_next);                 // see A, evaluated at the next position where y_next = py_next = 0.0 
          double v_next    = v + at;                                    // next speed prediction
          double epsi_next = psi_next - CppAD::atan(coeffs[1] + 2*coeffs[2]*px_next + 3*coeffs[3]*CppAD::pow(px_next, 2)); // see B, diff of 3rd order
          
          state << px_next, py_next, psi_next, v_next, cte_next, epsi_next;
          
          // Predict the optimal controls 
          // mpc_solution: [cost,steering,throttle,x0,y0,x1,y1,........xn,yn]
          // length: 2*N + 3
          double target_speed = expect_velocity;
          vector<double> mpc_solution = mpc.Solve(state, coeffs, target_speed);
          double cost = mpc_solution[0];
          double steer_value = mpc_solution[1];
          double throttle_value = mpc_solution[2];
          
          /*
               Output to Simulator
           */
		  if(throttle_value > 1 || throttle_value < -1)
		  {
		    throttle_value = throttle_value/fabs(throttle_value);
		  }
		  
		  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = steer_value/deg2rad(25);
		  old_outputs = {epsi_next, throttle_value};
		  
		  // message I/O
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // the points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          // Predict points
          int length = mpc_solution.size();
          for(int i=3;i<length;i+=2){
	        mpc_x_vals.push_back(mpc_solution[i]);
	        mpc_y_vals.push_back(mpc_solution[i+1]);
		  }
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          // the points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // Waypoints provided by the simulator
          // copy from the eigen vector to the std vector
          vector<double> next_x_vals(&ptsx_v[0], ptsx_v.data()+ptsx_v.cols()*ptsx_v.rows());
          vector<double> next_y_vals(&ptsy_v[0], ptsy_v.data()+ptsy_v.cols()*ptsy_v.rows());

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          
          /* 
               Control Latency 
          */
          
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(milli_latency));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          loop_count++;
          //std::cout << "loop: " << loop_count << std::endl;
          std::cout << "cost: " << cost << " steering: " << steer_value << " throttle: " << throttle_value << std::endl; 
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
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
