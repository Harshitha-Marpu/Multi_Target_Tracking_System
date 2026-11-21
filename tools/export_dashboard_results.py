"""
Create dashboard/validation_results.json from the results of option 9.

Run from the project root, after option 9 and evaluate_pipeline.py:
    python tools/export_dashboard_results.py
"""
import json
import re
import sqlite3
import sys
from pathlib import Path

REPORT = Path("radar_processing_report.txt")
EVAL = Path("evaluation_summary.json")
DB = Path("database/radar_tracking.db")
OUT = Path("dashboard/validation_results.json")


def read_report():
    values = {}
    for line in REPORT.read_text(encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def source_constant(path, name):
    match = re.search(rf"#define\s+{name}\s+([0-9.]+)", Path(path).read_text())
    return float(match.group(1)) if match else None


def main():
    for p in (REPORT, EVAL, DB):
        if not p.exists():
            sys.exit(f"Missing file: {p}")

    rep = read_report()
    ev = json.loads(EVAL.read_text())

    total = int(rep["Total detections"])
    processed = int(rep["Processed detections"])
    unprocessed = int(rep["Unclustered or unprocessed detections"])

    con = sqlite3.connect(DB)
    q = lambda sql: con.execute(sql).fetchone()[0]
    database = {
        "latest_records": q("SELECT COUNT(*) FROM radar_target_latest"),
        "history_records": q("SELECT COUNT(*) FROM radar_target_history"),
        "unique_target_ids": q("SELECT COUNT(DISTINCT target_id) FROM radar_target_history"),
        "latest_with_reference_id": q(
            "SELECT COUNT(*) FROM radar_target_latest WHERE reference_track_id <> ''"
        ),
        "history_with_reference_id": q(
            "SELECT COUNT(*) FROM radar_target_history WHERE reference_track_id <> ''"
        ),
    }
    con.close()

    result = {
        "dataset": {
            "name": "RadarScenes",
            "sequence": rep.get("Sequence", "sequence_151"),
            "total_detections": total,
            "labelled_dynamic_detections": ev["gt_detections"],
            "reference_objects": ev["gt_objects"],
        },
        "pipeline": {
            "frames": int(rep["Radar frames"]),
            "clustered_detections": processed,
            "noise_detections": unprocessed,
            "noise_percent": round(100.0 * unprocessed / total, 1),
            "tracks_created": int(rep["Total tracks created"]),
            "cluster_epsilon_m": float(rep["Clustering epsilon"].split()[0]),
            "cluster_min_points": int(rep["Minimum cluster points"]),
            "association_gate_m": source_constant("src/main.c", "ASSOCIATION_GATE"),
            "max_track_gap_s": source_constant("src/tracker.c", "MAX_TRACK_GAP"),
        },
        "database": database,
        "evaluation": {
            "gt_in_tracks": ev["gt_in_tracks"],
            "coverage_percent": ev["coverage_percent"],
            "idf1_percent": ev["idf1_percent"],
            "idr_percent": ev["idr_percent"],
            "idp_percent": ev["idp_percent"],
            "continuity_percent": ev["continuity_percent"],
            "identity_switches": ev["identity_switches"],
            "mixed_tracks": ev["mixed_tracks"],
            "weighted_purity_percent": ev["weighted_purity_percent"],
        },
        "per_object": ev["per_object"],
        "notes": [
            "Metrics are identity based. MOTA/MOTP are not computed because RadarScenes has no per-frame object positions.",
            "Tracking is run separately for each radar sensor, so one object can legitimately have several track IDs.",
            "Results are for a single sequence with five labelled moving objects and should not be generalised.",
        ],
    }

    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
