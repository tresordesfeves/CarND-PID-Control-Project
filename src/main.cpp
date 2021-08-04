//https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981
// mkdir build && cd build to create and enter the build directory
// cmake .. && make to compile your project
// ./pid to run your code
//pid_s.Init(0.134611, 0.000270736, 3.05349);
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <algorithm>    // std::min, std::max
#include <math.h>   //fabs
#include <vector>
// for convenience
using nlohmann::json;
using std::string;
using std::min;
using std::max;
using std::vector;


//global
    int twiddle_runs=0;//twiddle
    double p_histo=0;//twiddle
    double dp=1;//twiddle
    double best_error;//twiddle
    int scenario=1;
    //vector<double> dp={1,1,1};
    //vector<double> p ={0,0,0};


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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
/*
  int twiddle_runs=0;//twiddle
  double p_histo=0.08;//twiddle
  double dp=1;//twiddle
  double best_error;//twiddle
*/
  //pid::Init()
 // pid.Init(0.08, -0.001, 0.0); 
  
 // pid.Init(0.08, 0.0, 0.1); // drift right
 // pid.Init(0.08, 0.001, 0.5); // zig zag
  //pid.Init(0.08, 0.001, 0.05);//zig zag smoother
  //pid.Init(0.085, 0.001, 0.020); zig zag smoother
  //pid.Init(0.08,0.008,0.5);zig zag faster
  
  //pid.Init(0.08,0,0); zig zag after 25 seconds
  
  //pid.Init(0.08,0,0.001);after 30 s
  //pid.Init(0.08,0,0.0001);same
  //pid.Init(0.08,0.0001,0.001); a bit better
    
    //pid.Init(0.08,-0.001,0.005);pull to the right
   //pid.Init(0.08,0.0,1); best so far
 //pid.Init(0.08,0.001,1); best so far

  
  //pid.Init(0.08,-0.0001,0.001);
  //pid_s.Init(0.085, 0.0003, 1.35);
  //pid.Init(0.08, 0.001,1.05); not too shaby 
  //pid.Init(0.08, 0.01,1.05);/way to much zigzag
    //pid.Init(0.08, 0.0008,1.05);//best one
  		//pid.Init(0.08, 0.0008,1.1);
  //pid.Init(0.08, 0.0008,1.2); lets try 1.3
  //pid.Init(0.08, 0.0008,1.3); still shaving
  
  //pid.Init(0.085, 0.001,1.3);still shaving
  
 pid.Init(0.085, 0.001,1.5);
  
  
 

  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          pid.UpdateError(cte);
        
        
        // introducing modification for TWIDDLE
          /*

  

          if (twiddle_runs==0)
            {
              best_error=fabs(cte);

              pid.Update_Ks(p_histo+dp,0.0,0.0);

              twiddle_runs=1;
              scenario=1;
            }
          else
          {
            
     
              if (twiddle_runs<100)
                {
                  twiddle_runs+=1; //update the number of times twiddle was ran 
                  //only working on Kp to test, will extend to other coeffs later                 

                  switch (scenario)
                   {
                      case 1:
                        if (fabs(cte)<=best_error)
                          {
                            std::cout << "S 1 (error< best) : best error b4 : " << best_error << " best error post : " << fabs(cte)<< " P_b4 : "<< p_histo << "P_post : " << p_histo+(dp) << " dp b4 : " << dp<<" dp post  : " << 1.1*dp<< std::endl;
                            best_error=fabs(cte);
                            p_histo+=dp;// we retain this value for

                            dp*=1.1; // heading to the right direction so increase dp 
                            //std::cout << "fabs(cte)<=best_error, dp*=1.1, scenario 1 "
                            //std::cout << "S 1 (error< best) : best error b4 : " << best_error << " best error post : " << cte << " P_b4 : "<< p_histo << "P_post : " << p_histo << " dp b4 : " << dp << " db post  : " << dp << std::endl;
                            //std::cout << "dpe: " << dp << " P_histo: " << p_histo << " scen: " << scenario << " best: " << best_error<<" err: " << cte<< std::endl;

                            pid.Update_Ks(p_histo+dp,0.0,0.0); // attempt with an increased dp
                          // NO SCENARIO CHANGE
                          }
                        else 
                          {
                            
                            pid.Update_Ks(p_histo-dp,0.0,0.0);
                            std::cout << "S 1 (error>> best) : best error b4 : " << best_error << " best error post : " << fabs(cte)<< " p-histo unchanged : "<< p_histo << " dp unchanged  : " << dp<< std::endl;
                            //std::cout << "2dpe: " << dp << " P_histo: " << p_histo << " scen: " << scenario << " best: " << best_error<<" err: " << cte<< std::endl;


                          // SCENARIO CHANGES TO 2
                            scenario=2;
                          }
                        break;
                      case 2:
                        
                        if (fabs(cte)<=best_error)
                          {
                            best_error=fabs(cte);
                            p_histo-=dp;// we retain this decreased value 
                            dp*=1.1; // heading to the right direction so increase dp 
                            pid.Update_Ks(p_histo+dp,0.0,0.0); // attempt with an increased dp
                            std::cout << "S 2 (error< best) : best error b4 : " << best_error << " best error post : " << fabs(cte)<< " P_b4 : "<< p_histo << "P_post : " << p_histo-dp<< " dp b4 : " << dp<<" db increased 1.2  : " << 1.1*dp<< std::endl;
                            //std::cout << "dpe: " << dp << " P_histo: " << p_histo << " scen: " << scenario << " best: " << best_error<<" err: " << cte<< std::endl;


                          // BACK to SCENARIO 1
                            scenario=1;
                          }
                        else 
                          {
                            
                            std::cout << "S 2 (error>> best) fail  : best error b4 : " << best_error << " best error post : " << fabs(cte)<< " p-histo unchanged : "<< p_histo <<  " dp b4 : " << dp<<" db DECREASED  O.9 : " <<0.9*dp<< std::endl;
                            dp*=0.9; //all attempts failed time to lower dp 
                            pid.Update_Ks(p_histo+dp,0.0,0.0);

                            //std::cout << "dpe: " << dp << " P_histo: " << p_histo << " scen: " << scenario << " best: " << best_error<<" err: " << cte<< std::endl;

                            scenario=1;
                          }
                        break;
                      
                    }//end of switch

      
                  //std::cout << "dpe: " << dp << " P_histo: " << p_histo 
                            //<< std::endl; 
              
                }// end of "if (twiddle_runs<100)"

            }//end of ELSE (twiddle_runs==0)


        */


          steer_value = pid.TotalError();

        
          steer_value=max((double)-1,min((double) 1,steer_value));


    
          
          // DEBUG
                  //std::cout << "dpe: " << dp << " P_histo: " << p_histo << " scen: " << scenario << " best: " << best_error<<" err: " << cte<< std::endl;

         std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
 
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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