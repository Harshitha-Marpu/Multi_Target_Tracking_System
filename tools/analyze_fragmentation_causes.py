
import pandas as pd
import numpy as np

# Load radar association log
df = pd.read_csv("radar_association_log_frame.csv")

# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()

# Convert columns to numeric
for col in ["Timestamp", "Sensor_ID", "Internal_ID", "X", "Y"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.dropna(
    subset=["Timestamp", "Sensor_ID", "Internal_ID", "X", "Y"]
)

# Focus on reference objects with multiple internal IDs
counts = df.groupby(
    ["Sensor_ID", "Reference_ID"]
)["Internal_ID"].nunique()

fragmented_keys = counts[counts > 1].index

results = []

for sensor, reference in fragmented_keys:

    group = df[
        (df["Sensor_ID"] == sensor)
        & (df["Reference_ID"] == reference)
    ].copy()

    # Summarize each internal track
    segments = []

    for internal_id, track in group.groupby("Internal_ID"):

        start_time = track["Timestamp"].min()
        end_time = track["Timestamp"].max()

        first_frame = track[
            track["Timestamp"] == start_time
        ]

        last_frame = track[
            track["Timestamp"] == end_time
        ]

        segments.append({
            "Internal_ID": int(internal_id),
            "Start_Time": start_time,
            "End_Time": end_time,
            "Start_X": first_frame["X"].mean(),
            "Start_Y": first_frame["Y"].mean(),
            "End_X": last_frame["X"].mean(),
            "End_Y": last_frame["Y"].mean(),
            "Detections": len(track)
        })

    segments = sorted(
        segments,
        key=lambda item: item["Start_Time"]
    )

    # Compare consecutive track segments
    for i in range(1, len(segments)):

        previous = segments[i - 1]
        current = segments[i]

        time_gap = (
            current["Start_Time"] - previous["End_Time"]
        )

        distance = np.sqrt(
            (current["Start_X"] - previous["End_X"]) ** 2
            + (current["Start_Y"] - previous["End_Y"]) ** 2
        )

        results.append({
            "Sensor_ID": sensor,
            "Reference_ID": reference,
            "Previous_ID": previous["Internal_ID"],
            "New_ID": current["Internal_ID"],
            "Previous_End": previous["End_Time"],
            "New_Start": current["Start_Time"],
            "Time_Gap": time_gap,
            "Position_Distance": distance,
            "Previous_Detections": previous["Detections"],
            "New_Detections": current["Detections"]
        })

# Create results table
result_df = pd.DataFrame(results)

print("\n==========================================")
print("       FRAGMENTATION CAUSE ANALYSIS")
print("==========================================")

print("Track transitions examined:", len(result_df))

if not result_df.empty:

    print("\nTIME GAP SUMMARY (seconds)")
    print(result_df["Time_Gap"].describe().round(3))

    print("\nPOSITION DISTANCE SUMMARY (metres)")
    print(result_df["Position_Distance"].describe().round(3))

    print("\nFIRST 25 TRACK TRANSITIONS")

    print(
        result_df.head(25).round(3).to_string(index=False)
    )

    result_df.to_csv(
        "fragmentation_cause_details.csv",
        index=False
    )

    print("\nSaved: fragmentation_cause_details.csv")

else:
    print("No fragmented transitions found.")

print("\nAnalysis completed.")
print("==========================================")
