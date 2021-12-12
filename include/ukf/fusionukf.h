#ifndef FUSIONUKF_H_
#define FUSIONUKF_H_

#include <Eigen/Dense>

#include "common/datapoint.h"
#include "ukf/measurementpredictor.h"
#include "ukf/settings.h"
#include "ukf/statepredictor.h"
#include "ukf/stateupdater.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionUKF {
 public:
  FusionUKF();
  void process(const DataPoint& data);
  VectorXd get() const;
  double get_nis() const;

 private:
  bool initialized;
  long long timestamp;
  double nis;

  VectorXd x = VectorXd(NX);
  MatrixXd P = MatrixXd(NX, NX);
  StatePredictor statePredictor;
  MeasurementPredictor measurementPredictor;
  StateUpdater stateUpdater;

  void initialize(const DataPoint& data);
  void update(const DataPoint& data);
};

#endif  // FUSIONUKF_H_
