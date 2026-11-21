
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "tracker.h"
#include "radar_reader.h"
#include "clustering.h"
#include "database.h"

/* SQLite helper forward declaration */
static void save_all_targets_to_database(
    const Target targets[],
    int count
);

#define RADAR_CSV_PATH "data/radarscenes/sequence_151/radar_detections.csv"
#ifndef ASSOCIATION_GATE
#define ASSOCIATION_GATE 5.0
#endif
#define MAX_SIMULATION_STEPS 1000
#define ASSOCIATION_LOG_PATH "radar_association_log_frame.csv"

/* ==========================================
   WAIT FUNCTION
   ========================================== */

void wait_one_second(void)
{
#ifdef _WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
}

/* ==========================================
   LOAD DEMO DATA
   ========================================== */

void load_demo_data(Target targets[], int *count)
{
    struct Observation {
        int id;
        double x;
        double y;
        double time;
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
            targets,
            count,
            data[i].id,
            data[i].x,
            data[i].y,
            data[i].time
        );
    }
}

/* ==========================================
   AUTOMATIC SIMULATION
   ========================================== */

void run_automatic_simulation(
    Target targets[],
    int *count
)
{
    int steps;

    printf("\nEnter number of simulation steps: ");

    if (scanf("%d", &steps) != 1 ||
        steps < 1 ||
        steps > MAX_SIMULATION_STEPS) {

        printf("Invalid number of steps.\n");
        return;
    }

    for (int i = 1; i <= steps; i++) {

        printf(
            "\n========== SIMULATION STEP %d ==========\n",
            i
        );

        simulate_target_step(targets, count, 1.0);

        display_targets(targets, *count);

        display_tracking_accuracy(targets, *count);

        export_targets_json(targets, *count);

        if (i < steps) {
            wait_one_second();
        }
    }

    printf("\nSimulation completed.\n");
}

/* ==========================================
   MEASUREMENT NOISE
   ========================================== */

double generate_measurement_noise(void)
{
    return ((double)rand() / RAND_MAX) * 4.0 - 2.0;
}

/* ==========================================
   KALMAN NOISE SIMULATION
   ========================================== */

void run_noise_simulation(
    Target targets[],
    int *count
)
{
    int steps;

    printf("\nEnter simulation steps (1-1000): ");

    if (scanf("%d", &steps) != 1 ||
        steps < 1 ||
        steps > MAX_SIMULATION_STEPS) {

        printf("Invalid input.\n");
        return;
    }

    if (*count == 0) {
        printf("No targets available.\n");
        return;
    }

    double true_x[MAX_TARGETS];
    double true_y[MAX_TARGETS];
    double true_vx[MAX_TARGETS];
    double true_vy[MAX_TARGETS];
    double timestamps[MAX_TARGETS];

    int original_count = *count;

    for (int i = 0; i < original_count; i++) {

        true_x[i] = targets[i].x;
        true_y[i] = targets[i].y;

        true_vx[i] = targets[i].vx;
        true_vy[i] = targets[i].vy;

        timestamps[i] = targets[i].timestamp;

        targets[i].total_raw_measurement_error = 0.0;
        targets[i].total_kalman_estimation_error = 0.0;

        targets[i].noise_comparison_count = 0;
    }

    for (int step = 1; step <= steps; step++) {

        printf(
            "\n========== NOISE STEP %d ==========\n",
            step
        );

        for (int i = 0; i < original_count; i++) {

            int id = targets[i].id;

            true_x[i] += true_vx[i];
            true_y[i] += true_vy[i];

            double measured_x =
                true_x[i] + generate_measurement_noise();

            double measured_y =
                true_y[i] + generate_measurement_noise();

            timestamps[i] += 1.0;

            update_target(
                targets,
                count,
                id,
                measured_x,
                measured_y,
                timestamps[i]
            );

            double raw_dx = measured_x - true_x[i];
            double raw_dy = measured_y - true_y[i];

            double raw_error = sqrt(
                raw_dx * raw_dx + raw_dy * raw_dy
            );

            int index = find_target(targets, *count, id);

            if (index < 0) {
                continue;
            }

            double filtered_dx =
                targets[index].kalman.state[0] - true_x[i];

            double filtered_dy =
                targets[index].kalman.state[1] - true_y[i];

            double kalman_error = sqrt(
                filtered_dx * filtered_dx +
                filtered_dy * filtered_dy
            );

            targets[index].total_raw_measurement_error += raw_error;

            targets[index].total_kalman_estimation_error +=
                kalman_error;

            targets[index].noise_comparison_count++;

            int samples =
                targets[index].noise_comparison_count;

            targets[index].average_raw_measurement_error =
                targets[index].total_raw_measurement_error / samples;

            targets[index].average_kalman_estimation_error =
                targets[index].total_kalman_estimation_error / samples;

            if (targets[index].average_raw_measurement_error > 0.000001) {

                targets[index].error_reduction_percent =
                    (
                        targets[index].average_raw_measurement_error -
                        targets[index].average_kalman_estimation_error
                    ) /
                    targets[index].average_raw_measurement_error * 100.0;
            }

            printf(
                "Target %d | Raw Error: %.2f | Kalman Error: %.2f\n",
                id,
                raw_error,
                kalman_error
            );
        }

        export_targets_json(targets, *count);

        if (step < steps) {
            wait_one_second();
        }
    }

    printf("\nNoise simulation completed.\n");
}

/* ==========================================
   SAVE CLUSTER ASSOCIATION RESULTS
   ========================================== */

static void save_cluster_associations(
    FILE *log_file,
    const RadarDetection detections[],
    int detection_count,
    const int detection_cluster_ids[],
    const int cluster_track_ids[]
)
{
    if (log_file == NULL ||
        detections == NULL ||
        detection_cluster_ids == NULL ||
        cluster_track_ids == NULL) {
        return;
    }

    for (int i = 0; i < detection_count; i++) {

        int cluster_id = detection_cluster_ids[i];

        /* Ignore noise points and unclustered detections. */

        if (cluster_id < 0) {
            continue;
        }

        int internal_id = cluster_track_ids[cluster_id];

        if (internal_id <= 0) {
            continue;
        }

        const RadarDetection *detection = &detections[i];

        /* Only labelled detections are used for evaluation. */

        if (detection->reference_track_id[0] == '\0' ||
            detection->label_id == 11) {
            continue;
        }

        fprintf(
            log_file,
            "%.6f,%d,%s,%d,%d,%.6f,%.6f\n",
            detection->timestamp,
            detection->sensor_id,
            detection->reference_track_id,
            internal_id,
            detection->label_id,
            detection->x,
            detection->y
        );
    }
}

/* ==========================================
   PROCESS COMPLETE RADAR FRAME
   ========================================== */

static int process_frame_buffer(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarDetection frame[],
    int frame_count,
    FILE *log_file
)
{
    if (frame == NULL || frame_count <= 0) {
        return 0;
    }

    /* Allocate cluster memory. */

    RadarCluster *clusters = calloc(
        (size_t)frame_count,
        sizeof(RadarCluster)
    );

    int *detection_cluster_ids = malloc(
        (size_t)frame_count * sizeof(int)
    );

    int *cluster_track_ids = malloc(
        (size_t)frame_count * sizeof(int)
    );

    if (clusters == NULL ||
        detection_cluster_ids == NULL ||
        cluster_track_ids == NULL) {

        printf("Memory allocation failed during clustering.\n");

        free(clusters);
        free(detection_cluster_ids);
        free(cluster_track_ids);

        return 0;
    }

    /* Initialize mapping arrays. */

    for (int i = 0; i < frame_count; i++) {
        detection_cluster_ids[i] = -1;
        cluster_track_ids[i] = -1;
    }

    /*
       STEP 1:
       Cluster detections by spatial proximity.
       Clustering implementation separates sensors
       and timestamps.
    */

    int cluster_count = cluster_radar_detections(
        frame,
        frame_count,
        clusters,
        frame_count,
        detection_cluster_ids,
        CLUSTER_EPSILON,
        CLUSTER_MIN_POINTS
    );

    /*
       STEP 2:
       Associate cluster centroids with existing tracks.
    */

    int processed_clusters = 0;

    if (cluster_count > 0) {

        processed_clusters = update_radar_clusters(
            targets,
            count,
            next_track_id,
            clusters,
            cluster_count,
            cluster_track_ids,
            ASSOCIATION_GATE
        );
    }

    /*
       STEP 3:
       Count detections that belong to valid clusters.
    */

    int processed_detections = 0;

    for (int i = 0; i < frame_count; i++) {

        int cluster_id = detection_cluster_ids[i];

        if (cluster_id >= 0 &&
            cluster_id < cluster_count &&
            cluster_track_ids[cluster_id] > 0) {

            processed_detections++;
        }
    }

    /*
       STEP 4:
       Save association results for evaluation.
    */

    save_cluster_associations(
        log_file,
        frame,
        frame_count,
        detection_cluster_ids,
        cluster_track_ids
    );

    /*
       STEP 5:
       Save target states to SQLite.
       One transaction is used for the complete frame.
    */

    if (*count > 0) {

        if (!save_targets_batch_to_database(targets, *count)) {

            printf("Warning: SQLite frame save failed.\n");
        }
    }

    /*
       STEP 6:
       Display frame processing information.
    */

    printf(
        "Frame detections: %d | Clusters: %d | "
        "Processed clusters: %d | Processed detections: %d\n",
        frame_count,
        cluster_count,
        processed_clusters,
        processed_detections
    );

    free(clusters);
    free(detection_cluster_ids);
    free(cluster_track_ids);

    return processed_detections;
}

/* ==========================================
   REAL RADAR DATASET PROCESSING
   ========================================== */

void process_real_radar_dataset(
    Target targets[],
    int *count
)
{
    RadarDetection detection;

    RadarDetection *frame = NULL;

    size_t frame_capacity = 0;

    int frame_count = 0;
    int next_track_id = 1;

    long total_detections = 0;
    long processed_detections = 0;
    long rejected_detections = 0;
    long total_frames = 0;

    double current_timestamp = 0.0;

    int frame_started = 0;

    FILE *report = fopen(
        "radar_processing_report.txt",
        "w"
    );

    FILE *log_file = fopen(
        ASSOCIATION_LOG_PATH,
        "w"
    );

    printf("\n========================================\n");
    printf("       REAL RADAR DATASET PROCESSING\n");
    printf("========================================\n");

    printf("Dataset path: %s\n", RADAR_CSV_PATH);

    if (report == NULL) {
        printf("Warning: Could not create processing report.\n");
    }

    if (log_file == NULL) {
        printf("Warning: Could not create association log.\n");
    }
    else {
        fprintf(
            log_file,
            "Timestamp,Sensor_ID,Reference_ID,Internal_ID,Label_ID,X,Y\n"
        );
    }

    /* Open the predefined CSV path. */

    if (!open_radar_csv(RADAR_CSV_PATH)) {

        printf("\nERROR: Could not open RadarScenes CSV.\n");
        printf("Check whether this file exists:\n");
        printf("%s\n", RADAR_CSV_PATH);

        if (report != NULL) {
            fclose(report);
        }

        if (log_file != NULL) {
            fclose(log_file);
        }

        return;
    }

    /* Reset previous target information. */

    memset(
        targets,
        0,
        sizeof(Target) * MAX_TARGETS
    );

    *count = 0;

    /* Start the database from a clean state so results from
       earlier runs do not mix with this run. */
    if (reset_tracking_database()) {
        printf("Previous tracking records cleared from database.\n");
    }
    else {
        printf("Warning: Could not clear previous database records.\n");
    }

    printf("\nCSV file opened successfully.\n");
    printf("Processing RadarScenes sequence_151...\n\n");

    /* Read detections one by one. */

    while (read_radar_detection(&detection)) {

        total_detections++;

        /* Initialize first frame. */

        if (!frame_started) {

            current_timestamp = detection.timestamp;

            frame_started = 1;
        }

        /* Detect timestamp change. */

        else if (fabs(
            detection.timestamp - current_timestamp
        ) > 0.000001) {

            /* Process the previous frame. */

            int result = process_frame_buffer(
                targets,
                count,
                &next_track_id,
                frame,
                frame_count,
                log_file
            );

            processed_detections += result;

            if (frame_count > result) {
                rejected_detections += frame_count - result;
            }

            total_frames++;

            /* Start a new frame. */

            frame_count = 0;

            current_timestamp = detection.timestamp;
        }

        /* Allocate or increase frame buffer. */

        if ((size_t)frame_count >= frame_capacity) {

            size_t new_capacity =
                (frame_capacity == 0)
                ? 256
                : frame_capacity * 2;

            if (new_capacity < frame_capacity ||
                new_capacity > ((size_t)-1) / sizeof(RadarDetection)) {

                printf("Frame buffer capacity overflow.\n");

                rejected_detections++;
                break;
            }

            RadarDetection *new_frame = realloc(
                frame,
                new_capacity * sizeof(RadarDetection)
            );

            if (new_frame == NULL) {

                printf("Memory allocation failed.\n");

                rejected_detections++;
                break;
            }

            frame = new_frame;

            frame_capacity = new_capacity;
        }

        /* Store detection in current frame. */

        frame[frame_count] = detection;

        frame_count++;

        /* Display first few detections. */

        if (total_detections <= 5) {
            display_radar_detection(&detection);
        }

        /* Display processing progress. */

        if (total_detections % 10000 == 0) {

            printf(
                "\nRead: %ld | Processed: %ld | "
                "Frames: %ld | Tracks: %d\n",
                total_detections,
                processed_detections,
                total_frames,
                *count
            );
        }
    }

    /* Process the final frame. */

    if (frame_count > 0) {

        int result = process_frame_buffer(
            targets,
            count,
            &next_track_id,
            frame,
            frame_count,
            log_file
        );

        processed_detections += result;

        if (frame_count > result) {
            rejected_detections += frame_count - result;
        }

        total_frames++;
    }

    /* Release resources. */

    free(frame);

    close_radar_csv();

    if (log_file != NULL) {

        fclose(log_file);

        printf(
            "\nAssociation log saved: %s\n",
            ASSOCIATION_LOG_PATH
        );
    }

    /* Export final target information. */

    export_targets_json(targets, *count);

    /* Display processing summary. */

    printf("\n========================================\n");
    printf("          RADAR PROCESSING SUMMARY\n");
    printf("========================================\n");

    printf(
        "Total detections read: %ld\n",
        total_detections
    );

    printf(
        "Processed detections: %ld\n",
        processed_detections
    );

    printf(
        "Unclustered or unprocessed detections: %ld\n",
        rejected_detections
    );

    printf(
        "Radar frames processed: %ld\n",
        total_frames
    );

    printf(
        "Total tracks created: %d\n",
        next_track_id - 1
    );

    printf(
        "Track slots in memory (inactive slots are reused): %d\n",
        *count
    );

    printf(
        "Clustering epsilon: %.2f metres\n",
        (double)CLUSTER_EPSILON
    );

    printf(
        "Minimum cluster points: %d\n",
        CLUSTER_MIN_POINTS
    );

    /* Write processing report. */

    if (report != NULL) {

        fprintf(report, "RADAR PROCESSING REPORT\n");
        fprintf(report, "=======================\n");

        fprintf(report, "Dataset: RadarScenes\n");
        fprintf(report, "Sequence: sequence_151\n");

        fprintf(
            report,
            "Clustering epsilon: %.2f metres\n",
            (double)CLUSTER_EPSILON
        );

        fprintf(
            report,
            "Minimum cluster points: %d\n",
            CLUSTER_MIN_POINTS
        );

        fprintf(
            report,
            "Total detections: %ld\n",
            total_detections
        );

        fprintf(
            report,
            "Processed detections: %ld\n",
            processed_detections
        );

        fprintf(
            report,
            "Unclustered or unprocessed detections: %ld\n",
            rejected_detections
        );

        fprintf(
            report,
            "Radar frames: %ld\n",
            total_frames
        );

        fprintf(
            report,
            "Total tracks created: %d\n",
            next_track_id - 1
        );

        fprintf(
            report,
            "Track slots in memory (inactive slots are reused): %d\n",
            *count
        );

        fclose(report);

        printf("\nReport saved: radar_processing_report.txt\n");
    }

    /* Display final tracked targets. */

    display_targets(targets, *count);

    /* Save final dataset targets to SQLite. */

    save_all_targets_to_database(targets, *count);

    printf("\nRadar processing completed.\n");
}

/* ==========================================
   SAVE TARGETS TO SQLITE DATABASE
   ========================================== */

static void save_all_targets_to_database(
    const Target targets[],
    int count
)
{
    if (count <= 0) {
        return;
    }

    if (save_targets_batch_to_database(targets, count)) {

        printf(
            "\nSQLite: saved %d target records successfully.\n",
            count
        );
    }
    else {

        printf("\nSQLite: batch save failed.\n");
    }
}

/* ==========================================
   MAIN FUNCTION
   ========================================== */

int main(void)
{
    /*
       Static allocation prevents the large
       target array from consuming the stack.
    */

    static Target targets[MAX_TARGETS];

    int count = 0;
    int choice;

    srand((unsigned int)time(NULL));

    memset(targets, 0, sizeof(targets));

    /* Initialize SQLite database. */

    if (!initialize_database()) {

        printf("SQLite database initialization failed.\n");

        return 1;
    }

    /* Load demo targets. */

    load_demo_data(targets, &count);

    export_targets_json(targets, count);

    /* Save initial demo targets to SQLite. */

    save_all_targets_to_database(targets, count);

    printf("\n========================================\n");
    printf("       MULTI-TARGET TRACKING SYSTEM\n");
    printf("========================================\n");

    while (1) {

        printf("\n------------- MAIN MENU -------------\n");

        printf("1. Display all targets\n");
        printf("2. Add or update target\n");
        printf("3. View target history\n");
        printf("4. Predict future positions\n");
        printf("5. Exit\n");
        printf("6. Start automatic simulation\n");
        printf("7. View tracking accuracy report\n");
        printf("8. Run measurement noise simulation\n");
        printf("9. Process Real RadarScenes Dataset\n");

        printf("\nEnter your choice: ");

        if (scanf("%d", &choice) != 1) {

            printf("Invalid input. Exiting.\n");

            close_database();

            return 1;
        }

        switch (choice) {

            case 1:

                display_targets(targets, count);

                break;

            case 2: {

                int id;

                double x, y, timestamp;

                printf("Enter target ID: ");

                if (scanf("%d", &id) != 1) {
                    printf("Invalid target ID.\n");
                    break;
                }

                printf("Enter X position: ");

                if (scanf("%lf", &x) != 1) {
                    printf("Invalid X position.\n");
                    break;
                }

                printf("Enter Y position: ");

                if (scanf("%lf", &y) != 1) {
                    printf("Invalid Y position.\n");
                    break;
                }

                printf("Enter timestamp: ");

                if (scanf("%lf", &timestamp) != 1) {
                    printf("Invalid timestamp.\n");
                    break;
                }

                if (update_target(
                    targets,
                    &count,
                    id,
                    x,
                    y,
                    timestamp
                )) {

                    printf("Target updated successfully.\n");

                    export_targets_json(targets, count);

                    save_all_targets_to_database(targets, count);

                } else {

                    printf("Target update failed.\n");
                }

                break;
            }

            case 3: {

                int id;

                printf("Enter target ID: ");

                if (scanf("%d", &id) != 1) {
                    printf("Invalid target ID.\n");
                    break;
                }

                display_database_target_history(id);

                break;
            }

            case 4: {

                double future_seconds;

                printf("Enter future time interval: ");

                if (scanf("%lf", &future_seconds) != 1) {
                    printf("Invalid time interval.\n");
                    break;
                }

                if (future_seconds >= 0) {

                    display_prediction(
                        targets,
                        count,
                        future_seconds
                    );

                } else {

                    printf("Enter a non-negative interval.\n");
                }

                break;
            }

            case 5:

                export_targets_json(targets, count);

                printf("Exiting application.\n");

                close_database();

                return 0;

            case 6:

                run_automatic_simulation(
                    targets,
                    &count
                );

                save_all_targets_to_database(targets, count);

                break;

            case 7:

                display_tracking_accuracy(
                    targets,
                    count
                );

                break;

            case 8:

                run_noise_simulation(
                    targets,
                    &count
                );

                save_all_targets_to_database(targets, count);

                break;

            case 9:

                process_real_radar_dataset(
                    targets,
                    &count
                );

                break;

            default:

                printf("Invalid choice. Try again.\n");

                break;
        }
    }

    return 0;
}
