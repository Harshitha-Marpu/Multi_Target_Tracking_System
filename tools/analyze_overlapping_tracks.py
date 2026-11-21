
import pandas as pd

# Load radar association log
df = pd.read_csv("radar_association_log_frame.csv")

# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()

df["Timestamp"] = pd.to_numeric(
    df["Timestamp"], errors="coerce"
)

df["Internal_ID"] = pd.to_numeric(
    df["Internal_ID"], errors="coerce"
)

df = df.dropna(
    subset=["Timestamp", "Internal_ID"]
)

results = []

# Examine each sensor and reference object separately
for (sensor, reference), group in df.groupby(
    ["Sensor_ID", "Reference_ID"]
):

    segments = []

    for internal_id, track in group.groupby("Internal_ID"):

        segments.append({
            "Internal_ID": int(internal_id),
            "Start_Time": track["Timestamp"].min(),
            "End_Time": track["Timestamp"].max(),
            "Detection_Count": len(track)
        })

    # Compare every pair of internal tracks
    for i in range(len(segments)):

        for j in range(i + 1, len(segments)):

            a = segments[i]
            b = segments[j]

            overlap_start = max(
                a["Start_Time"],
                b["Start_Time"]
            )

            overlap_end = min(
                a["End_Time"],
                b["End_Time"]
            )

            overlap_duration = (
                overlap_end - overlap_start
            )

            if overlap_duration >= 0:

                results.append({
                    "Sensor_ID": sensor,
                    "Reference_ID": reference,
                    "Internal_ID_1": a["Internal_ID"],
                    "Internal_ID_2": b["Internal_ID"],
                    "Start_1": a["Start_Time"],
                    "End_1": a["End_Time"],
                    "Start_2": b["Start_Time"],
                    "End_2": b["End_Time"],
                    "Overlap_Duration": overlap_duration
                })

# Display results
result_df = pd.DataFrame(results)

print("\n==========================================")
print("       OVERLAPPING TRACK ANALYSIS")
print("==========================================")

print("Overlapping internal track pairs:", len(result_df))

if not result_df.empty:

    result_df = result_df.sort_values(
        "Overlap_Duration",
        ascending=False
    )

    print("\nTOP 20 OVERLAPPING TRACK PAIRS")

    print(
        result_df.head(20).round(3).to_string(index=False)
    )

    result_df.to_csv(
        "overlapping_track_details.csv",
        index=False
    )

    print("\nSaved: overlapping_track_details.csv")

else:

    print("No overlapping track pairs found.")

print("\nAnalysis completed.")
print("==========================================")
