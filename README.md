# Multi-Target Radar Tracking System Using Kalman Filtering and C

A software radar-tracking application. It reads real automotive radar detections
(RadarScenes), groups nearby detections into clusters, associates the clusters with
tracks, estimates each track's position and velocity with a Kalman filter, stores the
results in SQLite and shows them in a web dashboard.

The project covers the **software** side of radar tracking only. It does not deal with
radar hardware, antennas or embedded electronics.

## Pipeline

```
RadarScenes CSV  ->  reader  ->  frame buffer (same timestamp)
                                      |
                          DBSCAN-style clustering (per sensor)
                                      |
                  track association (Kalman-predicted gate, greedy one-to-one)
                                      |
                       Kalman filter update (x, y, vx, vy)
                          /                        \
                   SQLite database          targets.json + validation_results.json
                                                         |
                                                  web dashboard
```

| Stage | File | What it does |
| --- | --- | --- |
| Reading | `src/radar_reader.c` | Parses the CSV, converts timestamps to seconds |
| Clustering | `src/clustering.c` | Groups detections of one sensor and one frame within 1.25 m (min. 2 points) and computes a centroid. A cluster receives a reference ID only if all labelled points in it agree |
| Association | `src/tracker.c` | Gates clusters against Kalman-predicted track positions (5 m), assigns nearest pairs first, starts new tracks for the rest, expires tracks after 1.5 s without a match |
| Filtering | `src/tracker.c` | Constant-velocity Kalman filter, state `[x, y, vx, vy]` |
| Storage | `src/database.c` | `radar_target_latest` (current state) and `radar_target_history` (all observations) |
| Orchestration | `src/main.c` | Menu; option 9 runs the real-dataset pipeline |
| Evaluation | `tools/evaluate_pipeline.py` | Identity metrics against the RadarScenes ground-truth labels |
| Dashboard | `dashboard/` | Radar view, tables, Kalman analysis, prediction and validation results |

## Build and run

Requires GCC and the SQLite development library (the commands below are for MSYS2 UCRT64 on Windows).

```bash
gcc -std=c11 -Wall -Wextra -Iinclude src/main.c src/tracker.c src/radar_reader.c src/clustering.c src/database.c -o radar_tracker.exe -lm -lsqlite3
./radar_tracker.exe
```

Menu option **9** processes `data/radarscenes/sequence_151/radar_detections.csv`
(roughly half a minute). It clears the tracking tables first, so every run starts clean.
It writes `radar_processing_report.txt`, `radar_association_log_frame.csv`,
`targets.json`, `dashboard/targets.json` and the SQLite database.

Then evaluate and refresh the dashboard data (Python 3 with pandas and scipy):

```bash
python tools/evaluate_pipeline.py
python tools/export_dashboard_results.py
```

Open `dashboard/index.html` with the VS Code Live Server extension.

The RadarScenes data is not stored in this repository. Download it from
https://zenodo.org/records/4559821 and place `sequence_151` under `data/radarscenes/`.

## Results on RadarScenes sequence_151

| Measurement | Value |
| --- | --- |
| Radar detections / frames | 345,410 / 3,691 |
| Detections placed in clusters | 72,546 (21.0%); the other 79.0% are treated as noise |
| Tracks created | 1,410 (all stored in SQLite; about 124 slots are kept in memory) |
| Labelled moving detections (ground truth) | 8,205 from 5 reference objects |
| Labelled detections that reach a track | 7,108 (86.6%) |
| Track purity | 100%, no track mixes two reference objects |
| Identity F1 (IDF1) | 42.0% (IDR 39.1%, IDP 45.2%) |
| Adjacent-detection continuity | 99.5%, 36 identity switches |

How to read this: tracks are **pure** (a track never follows two different objects),
but a moving object is **fragmented** into several tracks. The most heavily
labelled object (5,541 detections) is split into 29 tracks across the four sensors.
Continuity is high because it only compares neighbouring detections; IDF1 is the
stricter measure and is the one to quote. See `docs/VALIDATION_REPORT.md` for details,
the parameter sensitivity study and the causes of fragmentation.

## Limitations

- Results come from **one sequence with five labelled moving objects**. They are not a general benchmark.
- **MOTA and MOTP are not reported.** RadarScenes provides labelled detections, not per-frame object positions, so those metrics are not defined here.
- IDP is optimistic: unlabelled clutter that shares a track with a labelled object is not visible in the log.
- Each sensor is tracked separately. The same physical object seen by several sensors receives several IDs; there is no cross-sensor fusion.
- Association is greedy nearest-neighbour with a constant-velocity model. It has no covariance-based gating and no track-confirmation logic.
- The dashboard on GitHub Pages is a published snapshot. It does not connect to a running C program.
- The `reference_track_id` ground truth is used only for evaluation. It is never used to decide which detections belong together.

## Possible future work

Cross-sensor fusion; Mahalanobis (covariance) gating; a global assignment step (Hungarian algorithm) instead of greedy matching; track confirmation and deletion rules; evaluation on more RadarScenes sequences.

## Author

Joshna Manili
