#include <se3_dmp/se3_dmp.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "se3_dmp");

  std::shared_ptr<SE3_DMP> se3_dmp;
  se3_dmp.reset(new SE3_DMP());

  return 0;
}
