#include "FusionEstimator/Estimators/EstimatorPortN.h"

static void check(double actual, double expected) {
    if (fabs(actual-expected) > 1e-10) {
        fprintf(stderr, "prediction %.17g != %.17g\n", actual, expected);
        exit(1);
    }
}

static void trajectory(int count) {
    EstimatorPortN model;
    StateSpaceModel_Go2_Initialization(&model);
    model.Int_Par[ESTIMATOR_PARAM_TIMESTAMP_TRANSITION] = 1;
    model.EstimatedState99[1] = 2.;
    model.EstimatedState99[2] = 3.;
    double observation[9] = {0};
    for (int i = 1; i <= count; ++i) {
        const double t = (double)i / count;
        StateSpaceModel_Go2_EstimatorPort(observation, t, &model);
        check(model.EstimatedState99[0], 2*t+1.5*t*t);
        check(model.EstimatedState99[1], 2+3*t);
    }
    // Same timestamp must not predict twice, and a large gap must not cause
    // an unbounded extrapolation (existing gap handling remains in effect).
    StateSpaceModel_Go2_EstimatorPort(observation, 1., &model);
    check(model.EstimatedState99[0], 3.5);
    StateSpaceModel_Go2_EstimatorPort(observation, 2., &model);
    check(model.EstimatedState99[0], 3.5);
    StateSpaceModel_Go2_EstimatorPortTermination(&model);
}

int main(void) {
    trajectory(200);
    trajectory(500);
    EstimatorPortN measured, legacy;
    StateSpaceModel_Go2_Initialization(&measured);
    StateSpaceModel_Go2_Initialization(&legacy);
    measured.Int_Par[ESTIMATOR_PARAM_TIMESTAMP_TRANSITION] = 1;
    measured.EstimatedState99[1] = legacy.EstimatedState99[1] = 1.;
    double observation[9] = {0};
    const double times[] = {.002, .007, .011, .012, .020};
    for (int i = 0; i < 5; ++i) {
        StateSpaceModel_Go2_EstimatorPort(observation, times[i], &measured);
        StateSpaceModel_Go2_EstimatorPort(observation, times[i], &legacy);
        check(measured.EstimatedState99[0], times[i]);
        check(legacy.EstimatedState99[0], .005*(i+1));
    }
    StateSpaceModel_Go2_EstimatorPortTermination(&measured);
    StateSpaceModel_Go2_EstimatorPortTermination(&legacy);
    puts("Timestamp prediction: 200/500 Hz, irregular samples, duplicate/gap, and instance isolation passed");
    return 0;
}
