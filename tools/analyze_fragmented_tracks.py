
import pandas as pd

# Load radar association log
df = pd.read_csv("radar_association_log_frame.csv")

# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()

# Group observations by sensor and reference object
summary = (
    df.groupby(["Sensor_ID", "Reference_ID"])
    .agg(
        Internal_ID_Count=("Internal_ID", "nunique"),
        Internal_IDs=(
            "Internal_ID",
            lambda x: sorted(x.unique().tolist())
        ),
        Detection_Count=("Internal_ID", "size"),
        First_Time=("Timestamp", "min"),
        Last_Time=("Timestamp", "max")
    )
    .reset_index()
)

# Keep only fragmented reference objects
fragmented = summary[
    summary["Internal_ID_Count"] > 1
].copy()

# Sort by number of internal IDs
fragmented = fragmented.sort_values(
    "Internal_ID_Count",
    ascending=False
)

print("\n==========================================")
print("       FRAGMENTED TRACK ANALYSIS")
print("==========================================")

print("Total fragmented reference objects:", len(fragmented))

print("\nFRAGMENTED TRACK DETAILS:\n")

if fragmented.empty:
    print("No fragmented tracks found.")
else:
    print(fragmented.to_string(index=False))

# Save report
fragmented.to_csv(
    "fragmented_track_details.csv",
    index=False
)

print("\nSaved: fragmented_track_details.csv")
print("==========================================")
