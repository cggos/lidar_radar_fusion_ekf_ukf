#ifndef USAGECHECK_H_
#define USAGECHECK_H_

#include <stdlib.h>

#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "common/datapoint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_arguments(int argc, char* argv[]);

void check_files(std::ifstream& in_file, std::string& in_names, std::ofstream& out_file, std::string& out_name);

void print_EKF_data(const VectorXd& RMSE,
                    const std::vector<VectorXd>& estimations,
                    const std::vector<VectorXd>& ground_truths,
                    const std::vector<DataPoint>& all_sensor_data);

#endif /* USAGECHECK_H_ */
