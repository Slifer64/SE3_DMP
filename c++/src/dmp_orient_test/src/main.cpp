#include <dmp_orient_test/dmp_orient_test.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dmp_orient_test");

  std::shared_ptr<DMP_orient_test> dmp_orient_test;
  dmp_orient_test.reset(new DMP_orient_test());

  return 0;
}
