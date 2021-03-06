#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mphToMps = 0.44704; // Miles per hour to meter per second

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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      string sdata = string(data).substr(0, length);
      cout << sdata << endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
        string s = hasData(sdata);
        if (s != "") {
          auto j = json::parse(s);
          string event = j[0].get<string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            vector<double> ptsx = j[1]["ptsx"];
            vector<double> ptsy = j[1]["ptsy"];
            double px = j[1]["x"];
            double py = j[1]["y"];
            double psi = j[1]["psi"];
            double v = j[1]["speed"];
            double steer_value = j[1]["steering_angle"];
            double throttle_value = j[1]["throttle"];

            // Add latency
            // predict state in 100ms
            // Running the simulation using the vehicle model starting from the current state for the duration of the latency
            double latency = 0.135;

            // apply the kinematic model update equations
            px = px + (v * mphToMps) * cos(psi) * latency;
            py = py + (v * mphToMps) * sin(psi) * latency;
            psi = psi - (v * mphToMps) / 2.67 * steer_value * latency;
            // no model found for mathematical dependency between throttle value and acceleration. Therefore the latency of v is included here


            // Calculate given  waypoint map coordinates to vehicles coordinates
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for(int i = 0; i < ptsx.size(); i++)
            {
              next_x_vals.push_back( (cos(-psi) * (ptsx.at(i) - px)) - (sin(-psi) * (ptsy.at(i) - py)));
              next_y_vals.push_back( (sin(-psi) * (ptsx.at(i) - px)) + (cos(-psi) * (ptsy.at(i) - py)));
            }

            // Fit the polynomial to the waypoints. Outcoming are the coefficients.
            Eigen::VectorXd ptsx_matrix(ptsx.size());
            Eigen::VectorXd ptsy_matrix(ptsy.size());
            ptsx_matrix << next_x_vals[0], next_x_vals[1], next_x_vals[2], next_x_vals[3], next_x_vals[4], next_x_vals[5];
            ptsy_matrix << next_y_vals[0], next_y_vals[1], next_y_vals[2], next_y_vals[3], next_y_vals[4], next_y_vals[5];
            auto coeffs = polyfit(ptsx_matrix, ptsy_matrix, 3);



            // Prepare initial state. Since the waypoints are transformed to the vehicles coordiante, the current values for x,y and psi is zero.
            double current_x = 0;
            double current_y = 0;
            double current_psi = 0;

            // The initial cross track error is calculated by evaluating at polynomial at waypoints polynom f(current_x) and subtracting current_y.
            double cte = polyeval(coeffs, current_x) - current_y;
            // Due to the sign starting at 0, the orientation error is -f'(current_x).
            double epsi = - atan(coeffs[1]);
            Eigen::VectorXd state(6);
            state << current_x, current_y, current_psi, v, cte, epsi;

            // Call the mpc.Solve with current, initial state and the waypoint polynom coeffs
            auto vars = mpc.Solve(state, coeffs);

            // Get steering angle and throttle using MPC. Both are in between [-1, 1].
            steer_value = -(vars.at(0));
            throttle_value = (vars.at(1));

            json msgJson;
            // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
            // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;

            vector<double> mpc_x_vals;
            vector<double> mpc_y_vals;

            for(int i = 1; i < (vars.size()/2); i++)
            {
              mpc_x_vals.push_back( vars.at(2*i) );
              mpc_y_vals.push_back( vars.at(2*i+1) );
            }

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Green line

            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does actuate the commands instantly.
            //
            // Feel free to play around with this value but should be to drive
            // around the track with 100ms latency.
            //
            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
            // SUBMITTING.
            this_thread::sleep_for(chrono::milliseconds(100));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
