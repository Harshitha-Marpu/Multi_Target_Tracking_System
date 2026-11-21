
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "radar_reader.h"

#define MAX_LINE_LENGTH 2048
#define CSV_COLUMNS 14

static FILE *radar_file = NULL;
static double first_timestamp = 0.0;
static int timestamp_initialized = 0;

/* Remove newline and surrounding whitespace */
static char *trim(char *text)
{
    while (isspace((unsigned char)*text)) {
        text++;
    }

    char *end = text + strlen(text);

    while (end > text && isspace((unsigned char)end[-1])) {
        end--;
    }

    *end = '\0';
    return text;
}

/*
 * Split CSV while preserving empty fields.
 * This is important because some RadarScenes track_id values are empty.
 */
static int split_csv(char *line, char *fields[], int expected)
{
    int count = 0;
    char *start = line;

    for (char *p = line; ; p++) {

        if (*p == ',' || *p == '\0') {

            if (count >= expected) {
                return 0;
            }

            if (*p == ',') {
                *p = '\0';
                fields[count++] = trim(start);
                start = p + 1;
            } else {
                fields[count++] = trim(start);
                break;
            }
        }
    }

    return count;
}

/* Open radar CSV */
int open_radar_csv(const char *filename)
{
    close_radar_csv();

    radar_file = fopen(filename, "r");

    if (radar_file == NULL) {
        printf("ERROR: Cannot open radar CSV: %s\n", filename);
        return 0;
    }

    char header[MAX_LINE_LENGTH];

    if (fgets(header, sizeof(header), radar_file) == NULL) {
        printf("ERROR: CSV file is empty.\n");
        close_radar_csv();
        return 0;
    }

    first_timestamp = 0.0;
    timestamp_initialized = 0;

    printf("Radar CSV opened successfully.\n");

    return 1;
}

/* Read one radar detection */
int read_radar_detection(RadarDetection *detection)
{
    if (radar_file == NULL || detection == NULL) {
        return 0;
    }

    char line[MAX_LINE_LENGTH];

    while (fgets(line, sizeof(line), radar_file) != NULL) {

        char *fields[CSV_COLUMNS];

        int count = split_csv(
            line,
            fields,
            CSV_COLUMNS
        );

        if (count != CSV_COLUMNS) {
            printf("Warning: Skipping malformed CSV row.\n");
            continue;
        }

        double raw_timestamp = strtod(fields[0], NULL);

        /* RadarScenes timestamp is in microseconds */
        if (!timestamp_initialized) {
            first_timestamp = raw_timestamp;
            timestamp_initialized = 1;
        }

        detection->timestamp =
            (raw_timestamp - first_timestamp) / 1000000.0;

        detection->sensor_id = atoi(fields[1]);

        detection->range = strtod(fields[2], NULL);
        detection->azimuth = strtod(fields[3], NULL);
        detection->rcs = strtod(fields[4], NULL);

        detection->radial_velocity = strtod(fields[5], NULL);
        detection->compensated_velocity = strtod(fields[6], NULL);

        detection->x = strtod(fields[7], NULL);
        detection->y = strtod(fields[8], NULL);

        detection->sequence_x = strtod(fields[9], NULL);
        detection->sequence_y = strtod(fields[10], NULL);

        snprintf(
            detection->uuid,
            sizeof(detection->uuid),
            "%s",
            fields[11]
        );

        snprintf(
            detection->reference_track_id,
            sizeof(detection->reference_track_id),
            "%s",
            fields[12]
        );

        detection->label_id = atoi(fields[13]);

        return 1;
    }

    return 0;
}

/* Close radar CSV */
void close_radar_csv(void)
{
    if (radar_file != NULL) {
        fclose(radar_file);
        radar_file = NULL;
    }

    timestamp_initialized = 0;
}

/* Display one radar detection */
void display_radar_detection(
    const RadarDetection *detection
)
{
    if (detection == NULL) {
        return;
    }

    printf("\n========== RADAR DETECTION ==========\n");

    printf("Timestamp: %.6f seconds\n", detection->timestamp);
    printf("Sensor ID: %d\n", detection->sensor_id);

    printf("Range: %.3f m\n", detection->range);
    printf("Azimuth: %.6f radians\n", detection->azimuth);
    printf("RCS: %.3f dBsm\n", detection->rcs);

    printf("Radial velocity: %.3f m/s\n",
           detection->radial_velocity);

    printf("Compensated velocity: %.3f m/s\n",
           detection->compensated_velocity);

    printf("Position: (%.3f, %.3f) m\n",
           detection->x, detection->y);

    printf("UUID: %s\n", detection->uuid);

    printf("Reference track ID: %s\n",
           detection->reference_track_id);

    printf("Label ID: %d\n", detection->label_id);

    printf("=====================================\n");
}
