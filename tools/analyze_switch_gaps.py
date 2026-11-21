
import pandas as pd

# ==========================================
# IDENTITY SWITCH GAP ANALYSIS
# ==========================================

# Read radar association log
file_path = "radar_association_log_frame.csv"

try:
    df = pd.read_csv(file_path)

except FileNotFoundError:
    print("ERROR: radar_association_log_frame.csv not found.")
    print("Run this script from the main project folder.")
    raise SystemExit(1)


# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()


# Sort observations by sensor, reference ID and timestamp
df = df.sort_values(
    by=["Sensor_ID", "Reference_ID", "Timestamp"]
)


# Get previous internal ID and timestamp
grouped = df.groupby(
    ["Sensor_ID", "Reference_ID"]
)

df["Previous_Internal_ID"] = (
    grouped["Internal_ID"].shift(1)
)

df["Previous_Timestamp"] = (
    grouped["Timestamp"].shift(1)
)


# Calculate time difference between observations
df["Time_Gap"] = (
    df["Timestamp"] - df["Previous_Timestamp"]
)


# Identify identity switches
switches = df[
    df["Previous_Internal_ID"].notna()
    & (
        df["Internal_ID"]
        != df["Previous_Internal_ID"]
    )
].copy()


# Separate switches based on 0.5-second threshold
short_gap = switches[
    switches["Time_Gap"] <= 0.5
]

long_gap = switches[
    switches["Time_Gap"] > 0.5
]


# ==========================================
# DISPLAY RESULTS
# ==========================================

print("\n==========================================")
print("       IDENTITY SWITCH GAP ANALYSIS")
print("==========================================")

print("Total identity switches:", len(switches))

print(
    "Switches within 0.5 seconds:",
    len(short_gap)
)

print(
    "Switches after 0.5 seconds:",
    len(long_gap)
)


# Calculate percentages
total = len(switches)

if total > 0:

    short_percentage = (
        len(short_gap) / total
    ) * 100

    long_percentage = (
        len(long_gap) / total
    ) * 100

    print(
        f"\nShort-gap switch percentage: "
        f"{short_percentage:.2f}%"
    )

    print(
        f"Long-gap switch percentage: "
        f"{long_percentage:.2f}%"
    )


# Show first 20 switches
print("\nFIRST 20 IDENTITY SWITCHES")

if not switches.empty:

    print(
        switches[
            [
                "Timestamp",
                "Sensor_ID",
                "Reference_ID",
                "Previous_Internal_ID",
                "Internal_ID",
                "Time_Gap"
            ]
        ].head(20).to_string(index=False)
    )

else:

    print("No identity switches found.")


# Show sensor-wise switch count
print("\nSWITCHES BY SENSOR")

if not switches.empty:

    print(
        switches.groupby("Sensor_ID").size()
    )


# Save complete analysis
output_file = "identity_switch_gap_analysis.csv"

switches.to_csv(
    output_file,
    index=False
)

print("\n==========================================")
print("Analysis completed successfully.")
print("Output saved:", output_file)
print("==========================================")
