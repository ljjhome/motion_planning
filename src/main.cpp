/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {

  message<string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, "a_star");
  
  HybridAStar::Planner hy;
  ros::Rate r(1);
  while(ros::ok()){
    
    ros::Time t0 = ros::Time::now();
    hy.plan();
    ros::Time t1 = ros::Time::now();
    ros::Duration d2(t1 - t0);
    //std::cout << "calculated total planning time shot in ms: " << d2 * 1000 << std::endl;
      
      r.sleep();
      ros::spinOnce();

  }
   

  ros::spin();
  return 0;
}
