
#ifndef RADAR_READER_H
#define RADAR_READER_H

#include <stdio.h>
#include "target.h"

/* Radar detection information */
typedef struct {

    double timestamp;
    int sensor_id;

    double range;
    double azimuth;
    double rcs;

    double radial_velocity;
    double compensated_velocity;

    double x;
    double y;

    double sequence_x;
    double sequence_y;

    char uuid[40];
    char reference_track_id[40];

    int label_id;

} RadarDetection;


/* Open the radar CSV file */
int open_radar_csv(const char *filename);


/* Read the next radar detection */
int read_radar_detection(RadarDetection *detection);


/* Close the radar CSV file */
void close_radar_csv(void);


/* Display radar detection information */
void display_radar_detection(
    const RadarDetection *detection
);

#endif
