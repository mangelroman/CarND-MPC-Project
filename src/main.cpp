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

using namespace std;

// for convenience
using json = nlohmann::json;

const double LATENCY = 0.1;

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

double polyeval(const vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double dpolyeval(const vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i - 1) * i;
  }
  return result;
}

double d2polyeval(const vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = 2; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i - 2) * i * (i - 1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
vector<double> polyfit(const vector<double> &xvals, vector<double> &yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }

  auto Q = A.householderQr();
  auto result_xd = Q.solve(Eigen::Map<Eigen::VectorXd>(yvals.data(), yvals.size()));
  vector<double> result(result_xd.size());
  Eigen::Map<Eigen::VectorXd>(result.data(), result.size()) = result_xd;
  return result;
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    auto start = chrono::high_resolution_clock::now();
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

          double px       = j[1]["x"];
          double py       = j[1]["y"];
          double psi      = j[1]["psi"];
          double v        = j[1]["speed"];
          double steering = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          auto pre_delay = {px, py, psi, v};
          auto actuators = {steering, throttle};
          auto post_delay = mpc.delay(pre_delay, actuators, LATENCY);

          px = post_delay[0];
          py = post_delay[1];
          psi = post_delay[2];
          v = post_delay[3];

          assert(ptsx.size() == ptsy.size());
          // Translate and rotate points to the vehicle frame of reference
          double sin_psi = sin(-psi);
          double cos_psi = cos(-psi);
          for (int i = 0; i < ptsx.size(); i++) {
            double xd = ptsx[i] - px;
            double yd = ptsy[i] - py;

            ptsx[i] = xd * cos_psi - yd * sin_psi;
            ptsy[i] = xd * sin_psi + yd * cos_psi;
          }
          // Vehicle frame of reference
          px = py = psi = 0;
          auto coeffs = polyfit(ptsx, ptsy, 3);
          double cte = polyeval(coeffs, px);
          double epsi = psi - atan(dpolyeval(coeffs, px));

          double x_center = ptsx[ptsx.size() / 2];
          double radius = pow(1 + pow(dpolyeval(coeffs, x_center), 2), 1.5) / abs(d2polyeval(coeffs, x_center));

          //printf("STATE: x=%8f y=%8f psi=%8f v=%8f cte=%8f epsi=%8f r=%8f\n", px, py, psi, v, cte, epsi, radius);

          auto state = {px, py, psi, v, cte, epsi, radius};

          if (mpc.solve(state, coeffs)) {

            printf("steering=%8f throttle=%8f\n", mpc.steering_value, mpc.throttle_value);

            json msgJson;
            // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
            // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
            msgJson["steering_angle"] = mpc.steering_value;
            msgJson["throttle"] = mpc.throttle_value;

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Green line
            msgJson["mpc_x"] = mpc.ptsx;
            msgJson["mpc_y"] = mpc.ptsy;

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line
            msgJson["next_x"] = ptsx;
            msgJson["next_y"] = ptsy;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does actuate the commands instantly.
            //
            // Feel free to play around with this value but should be to drive
            // around the track with 100ms latency.
            //
            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
            // SUBMITTING.
            chrono::duration<double> elapsed = chrono::high_resolution_clock::now() - start;
            auto final_latency = chrono::duration<double>(LATENCY) - elapsed;
            printf("Elapsed: %.3fs Latency: %.3fs\n", elapsed.count(), final_latency.count());
            if (final_latency.count() > 0) {
              this_thread::sleep_for(final_latency);
            }
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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

  h.onConnection([&h,&mpc,&argc,&argv](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    if (argc > 1) {
      mpc.load_config(argv[1]);
    }
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
