#ifndef FUSIONUKF_H_
#define FUSIONUKF_H_

#include <Eigen/Dense>

#include "common/datapoint.h"

#include "ukf/measurementpredictor.h"
#include "ukf/statepredictor.h"
#include "ukf/stateupdater.h"
#include "ukf/settings.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionUKF{

  private:
    bool initialized;
    long long timestamp;
    double nis;
    VectorXd x = VectorXd(NX);
    MatrixXd P = MatrixXd(NX, NX);
    StatePredictor statePredictor;
    MeasurementPredictor measurementPredictor;
    StateUpdater stateUpdater;

    //PRIVATE FUNCTIONS
    void initialize(const DataPoint& data);
    void update(const DataPoint& data);

  public:
    FusionUKF();
    void process(const DataPoint& data);
    VectorXd get() const;
    double get_nis() const;
};

#endif //FUSIONUKF_H_
