#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PID.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg_to_rad(double x) { return x * pi() / 180; }
double rad_to_deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string has_data(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int closest_waypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int next_waypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = closest_waypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> get_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = next_waypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> get_XY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

void compute_spline_based_trajectory(const int &target_lane, const double &car_s, const double &car_x,
                       const double &car_y, const double &car_yaw, const vector<double> &previous_path_x,
                       const vector<double> &previous_path_y, const vector<double> &map_waypoints_x,
                       const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s,
                       const double &ref_velocity, vector<double> &next_x_vals, vector<double> &next_y_vals) {
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg_to_rad(car_yaw);

  const int prev_size = previous_path_x.size();


  if ( prev_size < 2 ) {

      const double prev_car_x = car_x - cos(car_yaw);
      const double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
  } else {

      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      const double ref_x_prev = previous_path_x[prev_size - 2];
      const double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
  }

  const double step_size = 0.6 * ref_velocity;
  vector<double> next_wp0 = get_XY(car_s + step_size, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = get_XY(car_s + step_size * 2.0, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = get_XY(car_s + step_size * 3.0, 2 + 4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for ( int i = 0; i < ptsx.size(); i++ ) {
    const double shift_x = ptsx[i] - ref_x;
    const double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }


  tk::spline s;
  s.set_points(ptsx, ptsy);

  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }


  const double target_x = 30.0;
  const double target_y = s(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {
    const double N = target_dist / (0.02 * ref_velocity / 2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // driving limits
  constexpr double MAX_VELOCITY = 49.5;
  constexpr double VELOCITY_DELTA = 0.6;
  constexpr double MIN_FOLLOWING_SPEED_FRACTION = 0.9;
  constexpr double LANE_CHANGE_SPEED_SPACE_FRACTION = 0.3;
  constexpr int NUM_LANES = 3;

  const double linux_parameters[] = {0.015,0,0.2};

  // runtime tracking variables
  PID pid;
  pid.init(linux_parameters);
  bool started_lane_change = false;
  int target_lane = 1;
  double ref_velocity = 0;

  h.onMessage([&VELOCITY_DELTA,&MAX_VELOCITY,&target_lane,&started_lane_change,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_velocity, &pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = has_data(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
          // Main car's localization Data
          const double car_x = j[1]["x"];
          const double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          const double car_d = j[1]["d"];
          const double car_yaw = j[1]["yaw"];
          const double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          const vector<double> previous_path_x = j[1]["previous_path_x"];
          const vector<double> previous_path_y = j[1]["previous_path_y"];
          const double end_path_s = j[1]["end_path_s"];

          // determine secondary state of car
          const int car_lane = car_d / 4.0;
          const double follow_distance = max(10.0, car_speed);
          const double lane_change_gap = max(10.0, car_speed * LANE_CHANGE_SPEED_SPACE_FRACTION);
          const bool lane_change_occuring = ((int)round(car_d) % 4) < 1;

          // detect lane changes in progress
          if (started_lane_change) {
            if (lane_change_occuring) {
              started_lane_change = false;
            }
          }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          const vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          const int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // determine if there are cars to the left or right, and how close they are
          double closest_car = std::numeric_limits<double>::max();
          bool car_left = false;
          bool car_right = false;
          double car_left_gap = std::numeric_limits<double>::max();
          double car_right_gap = std::numeric_limits<double>::max();
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
              const float d = sensor_fusion[i][6];
              const int other_lane = d / 4.0;
              const double vx = sensor_fusion[i][3];
              const double vy = sensor_fusion[i][4];
              const double other_speed = sqrt(vx*vx+vy*vy);
              double other_s = sensor_fusion[i][5];

              other_s += ((double) prev_size * 0.02 * other_speed);

              const double gap = other_s - car_s;

              if (other_lane == car_lane) {
                if (gap > 0 && closest_car > gap) {
                  closest_car = gap;
                }
              } else if (!car_left && other_lane == (car_lane - 1)) {
                // lane left of car
                if (-lane_change_gap < gap && gap < lane_change_gap) {
                  car_left = true;
                }
                if (0.0 < gap && gap < car_left_gap) {
                  car_left_gap = gap;
                }

              } else if (other_lane == (car_lane + 1)) {
                // lane right of car
                if (-lane_change_gap < gap && gap < lane_change_gap) {
                  car_right = true;
                }
                if (0.0 < gap && gap < car_right_gap) {
                  car_right_gap = gap;
                }
              }
          }

          // should the car attempt to change lanes?
          if (!started_lane_change && !lane_change_occuring && closest_car < MAX_VELOCITY && ref_velocity < MAX_VELOCITY * MIN_FOLLOWING_SPEED_FRACTION) {
            if (!car_left && car_left_gap > closest_car && ( // no car on left and left lane is more open ahead than center
                  (car_lane == NUM_LANES - 1 || (car_lane > 0 && car_right)) || // in farthest right lane or in center with right blocked
                  (car_lane > 0 && !car_right && car_left_gap >= car_right_gap))) { // in center lane with right unblocked and left more open ahead than right
              // shift left
              target_lane--;
              started_lane_change = true;
            } else if (car_lane < NUM_LANES - 1 && car_right_gap > closest_car && !car_right) { // not in right lane, right lane is open more ahead than center, right lane unblocked
              // shift right
              target_lane++;
              started_lane_change = true;
            }
          }

          // compute acceleration, smoothed by a PID controller and bounded max maximum acceleration for comfort
          const double acceleration = pid.compute_control_value(follow_distance - closest_car);
          const double trimmed_acceleration = min(max(-3.0 * VELOCITY_DELTA, acceleration), VELOCITY_DELTA);

          // compute new reference velocity bounded by 2.0 and speed limit
          ref_velocity = max(min(MAX_VELOCITY, ref_velocity + trimmed_acceleration), 5.0);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // compute trajectory based on target lane and velocity
          compute_spline_based_trajectory(target_lane, car_s, car_x, car_y, car_yaw, previous_path_x,
                     previous_path_y, map_waypoints_x,
                     map_waypoints_y, map_waypoints_s,
                     ref_velocity, next_x_vals, next_y_vals);

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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


