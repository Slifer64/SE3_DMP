#include <dmp_pos_test/dmp_pos_test.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dmp_pos_test");

  std::shared_ptr<DMP_pos_test> dmp_pos_test;
  dmp_pos_test.reset(new DMP_pos_test());

  return 0;
}
