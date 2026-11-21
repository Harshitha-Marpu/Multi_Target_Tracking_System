#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "radar_reader.h"
#include "target.h"

#ifndef CLUSTER_EPSILON
#define CLUSTER_EPSILON 1.25
#endif
#ifndef CLUSTER_MIN_POINTS
#define CLUSTER_MIN_POINTS 2
#endif

typedef struct {
    double x;
    double y;
    double timestamp;

    int sensor_id;

    double range;
    double azimuth;
    double rcs;
    double radial_velocity;

    int detection_count;

    char reference_track_id[MAX_REFERENCE_ID_LENGTH];

} RadarCluster;

int cluster_radar_detections(
    const RadarDetection detections[],
    int detection_count,
    RadarCluster clusters[],
    int max_clusters,
    int detection_cluster_ids[],
    double epsilon,
    int min_points
);

#endif