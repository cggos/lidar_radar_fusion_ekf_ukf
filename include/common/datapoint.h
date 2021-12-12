#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include <stdlib.h>

#include <Eigen/Dense>
#include <iostream>

#include "common/tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class DataPointType { LIDAR, RADAR, STATE, TRUTH };

class DataPoint {
 public:
  DataPoint();

  DataPoint(const long long timestamp, const DataPointType& data_type, const VectorXd& raw);

  void set(const long long timestamp, const DataPointType& data_type, const VectorXd& raw);

  VectorXd get() const;

  VectorXd get_state() const;

  VectorXd get_vec() const;

  DataPointType get_type() const;

  long long get_timestamp() const;

  void print() const;

 private:
  long long timestamp;
  bool initialized;
  DataPointType data_type;
  VectorXd raw;
};

#endif /* DATAPOINT_H_*/
