#ifndef SE3_DMP_UTILS_H
#define SE3_DMP_UTILS_H

#define PACKAGE_NAME "se3_dmp"

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include <armadillo>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

#endif // SE3_DMP_UTILS_H
