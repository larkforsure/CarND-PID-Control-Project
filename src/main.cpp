#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.1415926
#endif

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Controlling steering angle
  PID pid_steering("Steering");
  // Controlling throttle
  PID pid_throttle("Throttle");

  // On purpose don't give optimum values in the initialization 
  pid_steering.Init(0.2, 0.0005, 10.0);
  pid_throttle.Init(-0.35, -0.0004, 50.0);
  //pid_throttle.Init(2.0, 0.005, 100.0);

  h.onMessage([&pid_steering, &pid_throttle](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s.c_str());
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.;
		  double throttle_value = 0.;
		  bool achieved_optimum = false;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  pid_steering.UpdateError(cte);
		  steer_value = pid_steering.TotalError();
		  // clamp
		  steer_value = (steer_value > 1.) ? 1. : (steer_value < -1.) ? -1. : steer_value;
		 
		  pid_throttle.UpdateError(cte);
		  throttle_value = pid_throttle.TotalError();
		  // clamp
		  throttle_value = (throttle_value > 1.6) ? 1.6 : (throttle_value < 0.3) ? 0.3 : throttle_value;

		  achieved_optimum = pid_steering.UpdateTwiddle();
		  achieved_optimum &= pid_throttle.UpdateTwiddle();

		  // DEBUG
		  if (achieved_optimum)
			  std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle value: " << throttle_value << std::endl;

		  // When car gets out of track again, reset dp values to re-caliberate
		  if (achieved_optimum && fabs(cte) > 1) {
			  // Useless ,will always fall back to the first optimum values and increase the instability
			  pid_steering.Recaliberate();
			  pid_throttle.Recaliberate();
		  }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle_value; // 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
