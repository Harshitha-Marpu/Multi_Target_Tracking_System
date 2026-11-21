
#include <stdio.h>
#include <string.h>

#include "clustering.h"

int main(void)
{
    RadarDetection detections[5] = {0};

    /* Four nearby radar detections. */
    detections[0].x = 5.065028;
    detections[0].y = 3.537327;

    detections[1].x = 4.286916;
    detections[1].y = 3.790282;

    detections[2].x = 4.673667;
    detections[2].y = 3.781026;

    detections[3].x = 5.445622;
    detections[3].y = 3.623694;

    /* One distant detection. */
    detections[4].x = 20.0;
    detections[4].y = 20.0;

    for (int i = 0; i < 5; i++) {
        detections[i].timestamp = 0.843490;
        detections[i].sensor_id = 3;
        detections[i].range = 6.0;
        detections[i].azimuth = 0.5;
        detections[i].rcs = 10.0;
        detections[i].compensated_velocity = 2.0;
    }

    RadarCluster clusters[5] = {0};

    int detection_cluster_ids[5];

    int cluster_count = cluster_radar_detections(
        detections,
        5,
        clusters,
        5,
        detection_cluster_ids,
        CLUSTER_EPSILON,
        CLUSTER_MIN_POINTS
    );

    printf("\n========== CLUSTERING TEST ==========\n");

    printf("Input detections: 5\n");
    printf("Clusters created: %d\n", cluster_count);

    for (int i = 0; i < cluster_count; i++) {
        printf("\nCluster %d\n", i + 1);

        printf("Centroid X: %.4f\n", clusters[i].x);
        printf("Centroid Y: %.4f\n", clusters[i].y);

        printf("Detection count: %d\n",
               clusters[i].detection_count);
    }

    printf("\nDetection-to-cluster mapping:\n");

    for (int i = 0; i < 5; i++) {
        printf(
            "Detection %d -> Cluster %d\n",
            i + 1,
            detection_cluster_ids[i]
        );
    }

    return 0;
}
