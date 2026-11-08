
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "tracker.h"

// Load demo observations
void load_demo_data(Target targets[], int *count)
{
    struct Observation {
        int id;
        double x, y, time;
    };

    struct Observation data[] = {
        {101, 10, 20, 0},
        {202, 50, 15, 0},
        {303, 25, 60, 0},

        {101, 12, 22, 1},
        {202, 49, 17, 1},
        {303, 27, 59, 1},

        {101, 14, 24, 2},
        {202, 48, 19, 2},
        {303, 29, 58, 2}
    };

    int total = sizeof(data) / sizeof(data[0]);

    for (int i = 0; i < total; i++) {
        update_target(
            targets, count,
            data[i].id,
            data[i].x,
            data[i].y,
            data[i].time
        );
    }
}

// Wait one second
void wait_one_second(void)
{
#ifdef _WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
}

// Automatic target simulation
void run_automatic_simulation(Target targets[], int *count)
{
    int steps;

    printf("\nEnter number of simulation steps: ");

    if (scanf("%d", &steps) != 1 || steps < 1 || steps > 1000) {
        printf("Enter a valid number between 1 and 1000.\n");
        return;
    }

    printf("\nStarting automatic target simulation...\n");

    for (int i = 1; i <= steps; i++) {
        printf("\n========== SIMULATION STEP %d ==========\n", i);

        simulate_target_step(targets, count, 1.0);

        display_targets(targets, *count);
        display_tracking_accuracy(targets, *count);

        export_targets_json(targets, *count);

        printf("Dashboard data updated.\n");

        if (i < steps) {
            wait_one_second();
        }
    }

    printf("\nAutomatic simulation completed.\n");
}

// Generate random measurement noise between -2 and +2
double generate_measurement_noise(void)
{
    return ((double)rand() / RAND_MAX) * 4.0 - 2.0;
}

// Measurement noise simulation
void run_noise_simulation(Target targets[], int *count)
{
    int steps;

    printf("\n========== MEASUREMENT NOISE SIMULATION ==========\n");

    printf("Enter number of simulation steps (1-1000): ");

    if (scanf("%d", &steps) != 1 || steps < 1 || steps > 1000) {
        printf("Invalid input. Enter a number between 1 and 1000.\n");
        return;
    }

    if (*count == 0) {
        printf("No targets available for simulation.\n");
        return;
    }

    double true_x[MAX_TARGETS];
    double true_y[MAX_TARGETS];

    double true_vx[MAX_TARGETS];
    double true_vy[MAX_TARGETS];

    double timestamp[MAX_TARGETS];

    // Initialize ground-truth positions and experiment metrics
    for (int i = 0; i < *count; i++) {
        true_x[i] = targets[i].x;
        true_y[i] = targets[i].y;

        true_vx[i] = targets[i].vx;
        true_vy[i] = targets[i].vy;

        timestamp[i] = targets[i].timestamp;

        targets[i].total_raw_measurement_error = 0.0;
        targets[i].total_kalman_estimation_error = 0.0;

        targets[i].average_raw_measurement_error = 0.0;
        targets[i].average_kalman_estimation_error = 0.0;
        targets[i].error_reduction_percent = 0.0;

        targets[i].noise_comparison_count = 0;
    }

    printf("\nStarting noisy measurement simulation...\n");
    printf("Measurement noise range: -2.0 to +2.0 units\n");

    for (int step = 1; step <= steps; step++) {

        printf("\n========== NOISE SIMULATION STEP %d ==========\n", step);

        for (int i = 0; i < *count; i++) {

            int id = targets[i].id;

            // Move the actual simulated target
            true_x[i] += true_vx[i];
            true_y[i] += true_vy[i];

            // Reflect at radar boundaries
            if (true_x[i] < 0.0 || true_x[i] > 100.0) {
                true_x[i] = true_x[i] < 0.0
                    ? -true_x[i] : 200.0 - true_x[i];

                true_vx[i] = -true_vx[i];
            }

            if (true_y[i] < 0.0 || true_y[i] > 100.0) {
                true_y[i] = true_y[i] < 0.0
                    ? -true_y[i] : 200.0 - true_y[i];

                true_vy[i] = -true_vy[i];
            }

            // Generate noisy radar measurements
            double measured_x = true_x[i] + generate_measurement_noise();
            double measured_y = true_y[i] + generate_measurement_noise();

            timestamp[i] += 1.0;

            // Apply Kalman filter through the C tracking engine
            update_target(
                targets,
                count,
                id,
                measured_x,
                measured_y,
                timestamp[i]
            );

            // Calculate raw measurement error
            double raw_error_x = measured_x - true_x[i];
            double raw_error_y = measured_y - true_y[i];

            double raw_error = sqrt(
                raw_error_x * raw_error_x +
                raw_error_y * raw_error_y
            );

            // Calculate Kalman estimation error
            double filtered_error_x =
                targets[i].kalman.state[0] - true_x[i];

            double filtered_error_y =
                targets[i].kalman.state[1] - true_y[i];

            double kalman_error = sqrt(
                filtered_error_x * filtered_error_x +
                filtered_error_y * filtered_error_y
            );

            // Store cumulative experiment results
            targets[i].total_raw_measurement_error += raw_error;

            targets[i].total_kalman_estimation_error += kalman_error;

            targets[i].noise_comparison_count++;

            // Calculate average errors
            targets[i].average_raw_measurement_error =
                targets[i].total_raw_measurement_error /
                targets[i].noise_comparison_count;

            targets[i].average_kalman_estimation_error =
                targets[i].total_kalman_estimation_error /
                targets[i].noise_comparison_count;

            // Calculate percentage of error reduction
            if (targets[i].average_raw_measurement_error > 0.000001) {
                targets[i].error_reduction_percent =
                    (
                        targets[i].average_raw_measurement_error -
                        targets[i].average_kalman_estimation_error
                    ) /
                    targets[i].average_raw_measurement_error * 100.0;
            } else {
                targets[i].error_reduction_percent = 0.0;
            }

            printf("\nTarget ID: %d\n", id);

            printf("Actual Position: (%.2f, %.2f)\n",
                   true_x[i], true_y[i]);

            printf("Noisy Measurement: (%.2f, %.2f)\n",
                   measured_x, measured_y);

            printf("Kalman Position: (%.2f, %.2f)\n",
                   targets[i].kalman.state[0],
                   targets[i].kalman.state[1]);

            printf("Raw Measurement Error: %.2f\n", raw_error);
            printf("Kalman Estimation Error: %.2f\n", kalman_error);
        }

        // Export latest target and experiment data
        export_targets_json(targets, *count);

        printf("\nDashboard data updated.\n");

        if (step < steps) {
            wait_one_second();
        }
    }

    // Final comparison report
    printf("\n============================================\n");
    printf(" KALMAN FILTER PERFORMANCE COMPARISON\n");
    printf("============================================\n");

    for (int i = 0; i < *count; i++) {

        printf("\nTarget ID: %d\n", targets[i].id);

        printf("Average Raw Measurement Error: %.2f\n",
               targets[i].average_raw_measurement_error);

        printf("Average Kalman Estimation Error: %.2f\n",
               targets[i].average_kalman_estimation_error);

        printf("Error Reduction: %.2f%%\n",
               targets[i].error_reduction_percent);
    }

    printf("\nNoise simulation completed.\n");
}

// Main application
int main(void)
{
    Target targets[MAX_TARGETS] = {0};

    int count = 0;
    int choice;

    srand((unsigned int)time(NULL));

    load_demo_data(targets, &count);

    export_targets_json(targets, count);

    printf("\n====================================\n");
    printf(" MULTI-TARGET TRACKING APPLICATION\n");
    printf("====================================\n");

    printf("Loaded %d simulated targets.\n", count);

    while (1) {

        printf("\n----------- MAIN MENU -----------\n");

        printf("1. Display all targets\n");
        printf("2. Add or update target\n");
        printf("3. View target history\n");
        printf("4. Predict future positions\n");
        printf("5. Exit\n");
        printf("6. Start automatic target simulation\n");
        printf("7. View tracking accuracy report\n");
        printf("8. Run measurement noise simulation\n");

        printf("\nEnter your choice: ");

        if (scanf("%d", &choice) != 1) {
            printf("Invalid input.\n");
            return 1;
        }

        if (choice == 1) {

            display_targets(targets, count);

        }

        else if (choice == 2) {

            int id;
            double x, y, timestamp;

            printf("Enter target ID: ");
            scanf("%d", &id);

            printf("Enter X position: ");
            scanf("%lf", &x);

            printf("Enter Y position: ");
            scanf("%lf", &y);

            printf("Enter timestamp: ");
            scanf("%lf", &timestamp);

            int existing_index = find_target(targets, count, id);

            int result = update_target(
                targets, &count, id, x, y, timestamp
            );

            if (result == 1) {
                if (existing_index == -1) {
                    printf("New target registered successfully.\n");
                } else {
                    printf("Target updated successfully.\n");
                }

                export_targets_json(targets, count);
            } else {
                printf("Target update failed.\n");
            }

        }

        else if (choice == 3) {

            int id;

            printf("Enter target ID: ");
            scanf("%d", &id);

            display_target_history(targets, count, id);

        }

        else if (choice == 4) {

            double time;

            printf("Enter future time interval: ");
            scanf("%lf", &time);

            if (time >= 0.0) {
                display_prediction(targets, count, time);
            } else {
                printf("Enter a non-negative time interval.\n");
            }

        }

        else if (choice == 5) {

            printf("Exiting application.\n");
            break;

        }

        else if (choice == 6) {

            run_automatic_simulation(targets, &count);

        }

        else if (choice == 7) {

            display_tracking_accuracy(targets, count);

        }

        else if (choice == 8) {

            run_noise_simulation(targets, &count);

        }

        else {

            printf("Invalid choice. Try again.\n");

        }
    }

    return 0;
}
