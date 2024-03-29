#ifndef MEASUREMENTPREDICTOR_H_
#define MEASUREMENTPREDICTOR_H_

#include <Eigen/Dense>

#include "common/datapoint.h"
#include "common/tools.h"
#include "ukf/settings.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 MeasurementPredictor is a class responsible for
 calculating the predicted measurement vector z, measurement covariance matrix S
 and sigma point matrix transformed to measurement space sigma_z
 based on a given predicted sigma points matrix sigma_x

 After calling the process() please call getS(), getz() and get_sigma()
 to get the corresponding calculated values
*******************************/

class MeasurementPredictor {
 public:
  MeasurementPredictor();
  void process(const MatrixXd& sigma_x, const DataPointType sensor_type);
  MatrixXd get_sigma() const;
  MatrixXd getS() const;
  VectorXd getz() const;

 private:
  int nz;
  DataPointType current_type;
  MatrixXd R;
  VectorXd z;
  MatrixXd S;
  MatrixXd sigma_z;

  void initialize(const DataPointType sensor_type);

  /**
   * @brief transform predicted sigma_x into measurement space
   *
   * @param sigma_x
   * @return MatrixXd
   */
  MatrixXd compute_sigma_z(const MatrixXd& sigma_x);

  /**
   * @brief the mean predicted measurement vector z
   *
   * @param sigma_z
   * @return MatrixXd
   */
  MatrixXd compute_z(const MatrixXd& sigma_z);

  /**
   * @brief the measurement covariance matrix S
   *
   * @param sigma_z
   * @param predicted_z
   * @return MatrixXd
   */
  MatrixXd compute_S(const MatrixXd& sigma_z, const MatrixXd& predicted_z);
};

#endif  // MEASUREMENTPREDICTOR_H_
