#ifndef TRACKER_H
#define TRACKER_H

#include "target.h"

int find_target(const Target targets[], int count, int id);

int update_target(Target targets[], int *count, int id,
                  double x, double y, double timestamp);

void display_targets(const Target targets[], int count);

void display_target_history(const Target targets[],
                            int count, int id);

void display_prediction(const Target targets[], int count,
                        double future_seconds);

void simulate_target_step(Target targets[],
                          int *count, double time_step);

void display_tracking_accuracy(const Target targets[], int count);

void export_targets_json(const Target targets[], int count);

void initialize_kalman(Target *target);

void update_kalman(Target *target,
                   double measured_x,
                   double measured_y,
                   double dt);

#endif