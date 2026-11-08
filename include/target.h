
#ifndef TARGET_H
#define TARGET_H

#define MAX_TARGETS 100
#define MAX_HISTORY 100

/* Target position history */
typedef struct {
    double x;
    double y;
    double timestamp;
} TrackPoint;

/* Kalman filter state */
typedef struct {
    double state[4];       // x, y, vx, vy
    double covariance[4][4];
    int initialized;
} KalmanState;

/* Complete target information */
typedef struct {

    int id;

    // Measured position
    double x;
    double y;
    double timestamp;

    // Filtered velocity and speed
    double vx;
    double vy;
    double speed;

    int active;

    // Prediction error information
    double prediction_error;
    double total_prediction_error;
    int prediction_count;
    int has_prediction_error;

    // Kalman filter
    KalmanState kalman;

    // Measurement noise experiment results
    double average_raw_measurement_error;
    double average_kalman_estimation_error;
    double error_reduction_percent;

    double total_raw_measurement_error;
    double total_kalman_estimation_error;

    int noise_comparison_count;

    // Target history
    TrackPoint history[MAX_HISTORY];
    int history_count;

} Target;

#endif
