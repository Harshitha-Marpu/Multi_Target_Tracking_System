
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "tracker.h"

#define MEASUREMENT_NOISE 4.0
#define PROCESS_NOISE 0.05
#define MIN_DT 0.000001

/* Add a position to target history */
static void append_history(Target *target,
                           double x,
                           double y,
                           double timestamp)
{
    if (target->history_count < MAX_HISTORY) {
        int index = target->history_count;

        target->history[index].x = x;
        target->history[index].y = y;
        target->history[index].timestamp = timestamp;

        target->history_count++;
    } else {
        for (int i = 1; i < MAX_HISTORY; i++) {
            target->history[i - 1] = target->history[i];
        }

        target->history[MAX_HISTORY - 1].x = x;
        target->history[MAX_HISTORY - 1].y = y;
        target->history[MAX_HISTORY - 1].timestamp = timestamp;
    }
}

/* Find a target using its ID */
int find_target(const Target targets[], int count, int id)
{
    for (int i = 0; i < count; i++) {
        if (targets[i].id == id) {
            return i;
        }
    }

    return -1;
}

/* Initialize Kalman filter */
void initialize_kalman(Target *target)
{
    memset(&target->kalman, 0, sizeof(KalmanState));

    target->kalman.state[0] = target->x;
    target->kalman.state[1] = target->y;
    target->kalman.state[2] = 0.0;
    target->kalman.state[3] = 0.0;

    for (int i = 0; i < 4; i++) {
        target->kalman.covariance[i][i] = 10.0;
    }

    target->kalman.initialized = 1;

    target->vx = 0.0;
    target->vy = 0.0;
    target->speed = 0.0;
}

/* Update Kalman filter */
void update_kalman(Target *target,
                   double measured_x,
                   double measured_y,
                   double dt)
{
    if (!target->kalman.initialized) {
        target->x = measured_x;
        target->y = measured_y;
        initialize_kalman(target);
        return;
    }

    if (dt <= MIN_DT) {
        dt = 1.0;
    }

    double F[4][4] = {
        {1.0, 0.0, dt,  0.0},
        {0.0, 1.0, 0.0, dt},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    double P[4][4];
    double FP[4][4] = {0};
    double predicted_P[4][4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = target->kalman.covariance[i][j];
        }
    }

    /* Predict state */
    double predicted_state[4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            predicted_state[i] +=
                F[i][j] * target->kalman.state[j];
        }
    }

    /* Calculate F * P */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    /* Calculate F * P * F transpose */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                predicted_P[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    /* Add process noise */
    for (int i = 0; i < 4; i++) {
        predicted_P[i][i] += PROCESS_NOISE * dt;
    }

    /* Measurement innovation */
    double innovation_x = measured_x - predicted_state[0];
    double innovation_y = measured_y - predicted_state[1];

    /* Innovation covariance */
    double S00 = predicted_P[0][0] + MEASUREMENT_NOISE;
    double S01 = predicted_P[0][1];
    double S10 = predicted_P[1][0];
    double S11 = predicted_P[1][1] + MEASUREMENT_NOISE;

    double determinant = S00 * S11 - S01 * S10;

    if (fabs(determinant) < 1e-12) {
        return;
    }

    /* Inverse of innovation covariance */
    double invS[2][2];

    invS[0][0] = S11 / determinant;
    invS[0][1] = -S01 / determinant;
    invS[1][0] = -S10 / determinant;
    invS[1][1] = S00 / determinant;

    /* Calculate Kalman gain */
    double K[4][2] = {0};

    for (int i = 0; i < 4; i++) {
        K[i][0] = predicted_P[i][0] * invS[0][0]
                + predicted_P[i][1] * invS[1][0];

        K[i][1] = predicted_P[i][0] * invS[0][1]
                + predicted_P[i][1] * invS[1][1];
    }

    /* Update filtered state */
    for (int i = 0; i < 4; i++) {
        target->kalman.state[i] =
            predicted_state[i]
            + K[i][0] * innovation_x
            + K[i][1] * innovation_y;
    }

    /* Update covariance */
    double updated_P[4][4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            updated_P[i][j] =
                predicted_P[i][j]
                - K[i][0] * predicted_P[0][j]
                - K[i][1] * predicted_P[1][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            target->kalman.covariance[i][j] = updated_P[i][j];
        }
    }

    /* Save filtered velocity and speed */
    target->vx = target->kalman.state[2];
    target->vy = target->kalman.state[3];

    target->speed = sqrt(
        target->vx * target->vx +
        target->vy * target->vy
    );
}

/* Add or update a target */
int update_target(Target targets[],
                  int *count,
                  int id,
                  double x,
                  double y,
                  double timestamp)
{
    int index = find_target(targets, *count, id);

    /* Register new target */
    if (index == -1) {
        if (*count >= MAX_TARGETS) {
            printf("Maximum target limit reached.\n");
            return 0;
        }

        index = *count;
        (*count)++;

        memset(&targets[index], 0, sizeof(Target));

        targets[index].id = id;
        targets[index].x = x;
        targets[index].y = y;
        targets[index].timestamp = timestamp;
        targets[index].active = 1;

        initialize_kalman(&targets[index]);

        append_history(&targets[index], x, y, timestamp);

        return 1;
    }

    Target *target = &targets[index];

    double dt = timestamp - target->timestamp;

    if (dt <= MIN_DT) {
        dt = 1.0;
    }

    /* Calculate previous constant-velocity prediction error */
    double predicted_x = target->x + target->vx * dt;
    double predicted_y = target->y + target->vy * dt;

    double error_x = x - predicted_x;
    double error_y = y - predicted_y;

    target->prediction_error =
        sqrt(error_x * error_x + error_y * error_y);

    target->total_prediction_error += target->prediction_error;
    target->prediction_count++;
    target->has_prediction_error = 1;

    /* Apply Kalman filter */
    update_kalman(target, x, y, dt);

    /* Save measured position */
    target->x = x;
    target->y = y;
    target->timestamp = timestamp;
    target->active = 1;

    append_history(target, x, y, timestamp);

    return 1;
}

/* Display target information */
void display_targets(const Target targets[], int count)
{
    printf("\n========== TARGET DETAILS ==========\n");

    for (int i = 0; i < count; i++) {
        printf("\nTarget ID: %d\n", targets[i].id);

        printf("Measured Position: (%.2f, %.2f)\n",
               targets[i].x, targets[i].y);

        printf("Kalman Position: (%.2f, %.2f)\n",
               targets[i].kalman.state[0],
               targets[i].kalman.state[1]);

        printf("Filtered Velocity: (%.2f, %.2f)\n",
               targets[i].vx, targets[i].vy);

        printf("Speed: %.2f\n", targets[i].speed);

        printf("Status: %s\n",
               targets[i].active ? "Active" : "Inactive");
    }
}

/* Display target history */
void display_target_history(const Target targets[],
                            int count,
                            int id)
{
    int index = find_target(targets, count, id);

    if (index == -1) {
        printf("Target not found.\n");
        return;
    }

    printf("\nHistory for Target %d:\n", id);

    for (int i = 0; i < targets[index].history_count; i++) {
        printf("Time: %.2f | Position: (%.2f, %.2f)\n",
               targets[index].history[i].timestamp,
               targets[index].history[i].x,
               targets[index].history[i].y);
    }
}

/* Predict future target positions */
void display_prediction(const Target targets[],
                        int count,
                        double future_seconds)
{
    printf("\n========== FUTURE POSITION PREDICTION ==========\n");

    for (int i = 0; i < count; i++) {
        double future_x =
            targets[i].kalman.state[0] +
            targets[i].kalman.state[2] * future_seconds;

        double future_y =
            targets[i].kalman.state[1] +
            targets[i].kalman.state[3] * future_seconds;

        printf("Target %d: Future Position = (%.2f, %.2f)\n",
               targets[i].id, future_x, future_y);
    }
}

/* Reflect position inside the 0–100 boundary */
static double reflect_position(double position)
{
    while (position < 0.0 || position > 100.0) {
        if (position < 0.0) {
            position = -position;
        }

        if (position > 100.0) {
            position = 200.0 - position;
        }
    }

    return position;
}

/* Simulate movement of all targets */
void simulate_target_step(Target targets[],
                          int *count,
                          double time_step)
{
    int original_count = *count;

    for (int i = 0; i < original_count; i++) {
        int id = targets[i].id;

        double next_x =
            targets[i].x + targets[i].vx * time_step;

        double next_y =
            targets[i].y + targets[i].vy * time_step;

        next_x = reflect_position(next_x);
        next_y = reflect_position(next_y);

        double next_timestamp =
            targets[i].timestamp + time_step;

        update_target(
            targets,
            count,
            id,
            next_x,
            next_y,
            next_timestamp
        );
    }
}

/* Display prediction error report */
void display_tracking_accuracy(const Target targets[], int count)
{
    printf("\n========== TRACKING ERROR REPORT ==========\n");

    for (int i = 0; i < count; i++) {
        double average_error = 0.0;

        if (targets[i].prediction_count > 0) {
            average_error =
                targets[i].total_prediction_error /
                targets[i].prediction_count;
        }

        printf("\nTarget ID: %d\n", targets[i].id);

        if (targets[i].has_prediction_error) {
            printf("Latest Prediction Error: %.2f\n",
                   targets[i].prediction_error);
        } else {
            printf("Latest Prediction Error: Not available\n");
        }

        printf("Average Prediction Error: %.2f\n", average_error);

        printf("Prediction Count: %d\n",
               targets[i].prediction_count);
    }
}

/* Write target data into an opened JSON file */
static void write_targets_json(FILE *file,
                               const Target targets[],
                               int count)
{
    fprintf(file, "{\n");
    fprintf(file, "    \"total_targets\": %d,\n", count);
    fprintf(file, "    \"targets\": [\n");

    for (int i = 0; i < count; i++) {
        double average_error = 0.0;

        if (targets[i].prediction_count > 0) {
            average_error =
                targets[i].total_prediction_error /
                targets[i].prediction_count;
        }

        fprintf(file, "        {\n");

        fprintf(file, "            \"id\": %d,\n", targets[i].id);

        /* Measured position */
        fprintf(file, "            \"x\": %.2f,\n", targets[i].x);
        fprintf(file, "            \"y\": %.2f,\n", targets[i].y);

        /* Filtered velocity */
        fprintf(file, "            \"vx\": %.2f,\n", targets[i].vx);
        fprintf(file, "            \"vy\": %.2f,\n", targets[i].vy);
        fprintf(file, "            \"speed\": %.2f,\n", targets[i].speed);

        /* Kalman-filtered position */
        fprintf(file, "            \"kalman_x\": %.2f,\n",
                targets[i].kalman.state[0]);

        fprintf(file, "            \"kalman_y\": %.2f,\n",
                targets[i].kalman.state[1]);

        /* Kalman-filtered velocity */
        fprintf(file, "            \"kalman_vx\": %.2f,\n",
                targets[i].kalman.state[2]);

        fprintf(file, "            \"kalman_vy\": %.2f,\n",
                targets[i].kalman.state[3]);

        /* Prediction error information */
        fprintf(file, "            \"prediction_error\": %.2f,\n",
                targets[i].prediction_error);

        fprintf(file, "            \"average_prediction_error\": %.2f,\n",
                average_error);

        fprintf(file, "            \"prediction_count\": %d,\n",
                targets[i].prediction_count);

        /* Measurement noise experiment results */
        fprintf(file, "            \"average_raw_measurement_error\": %.2f,\n",
                targets[i].average_raw_measurement_error);

        fprintf(file, "            \"average_kalman_estimation_error\": %.2f,\n",
                targets[i].average_kalman_estimation_error);

        fprintf(file, "            \"error_reduction_percent\": %.2f,\n",
                targets[i].error_reduction_percent);

        fprintf(file, "            \"noise_comparison_count\": %d,\n",
                targets[i].noise_comparison_count);

        fprintf(file, "            \"active\": %s\n",
                targets[i].active ? "true" : "false");

        if (i < count - 1) {
            fprintf(file, "        },\n");
        } else {
            fprintf(file, "        }\n");
        }
    }

    fprintf(file, "    ]\n");
    fprintf(file, "}\n");
}

/* Export data to both JSON files */
void export_targets_json(const Target targets[], int count)
{
    FILE *root_file = fopen("targets.json", "w");

    FILE *dashboard_file = fopen("dashboard/targets.json", "w");

    if (root_file == NULL) {
        printf("Error: Could not create root targets.json\n");
    } else {
        write_targets_json(root_file, targets, count);
        fclose(root_file);

        printf("Updated: targets.json\n");
    }

    if (dashboard_file == NULL) {
        printf("Error: Could not create dashboard/targets.json\n");
    } else {
        write_targets_json(dashboard_file, targets, count);
        fclose(dashboard_file);

        printf("Updated: dashboard/targets.json\n");
    }
}
