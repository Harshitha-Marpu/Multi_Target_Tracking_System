
import pandas as pd

# Read radar association log
file_path = "radar_association_log_frame.csv"

df = pd.read_csv(file_path)

# Remove unlabelled detections
df = df[df["Reference_ID"].notna()]
df = df[df["Label_ID"] != 11]

# Sort by sensor, reference ID and timestamp
df = df.sort_values(
    by=["Sensor_ID", "Reference_ID", "Timestamp"]
)

total_rows = len(df)

total_transitions = 0
continuous_associations = 0
identity_switches = 0

# Evaluate each sensor and reference object separately
for (sensor, reference), group in df.groupby(
    ["Sensor_ID", "Reference_ID"]
):

    internal_ids = group["Internal_ID"].tolist()

    for i in range(1, len(internal_ids)):

        total_transitions += 1

        if internal_ids[i] == internal_ids[i - 1]:
            continuous_associations += 1
        else:
            identity_switches += 1

# Calculate continuity percentage
if total_transitions > 0:
    continuity = (
        continuous_associations / total_transitions
    ) * 100
else:
    continuity = 0

print("\n========== TRACKING IDENTITY EVALUATION ==========")

print("Total Log Rows:", total_rows)
print("Total Transitions:", total_transitions)
print("Continuous Associations:", continuous_associations)
print("Identity Switches:", identity_switches)

print(f"Identity Continuity: {continuity:.2f}%")

print("=================================================")
