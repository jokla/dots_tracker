#include <iostream>

#include "VScam.h"

#include "ros/ros.h"




int main(int argc, char** argv)
{
  ros::init(argc, argv, "visp_cam_vs");
  
  VScam ic;
  ic.spin ();

  return 0;
}
