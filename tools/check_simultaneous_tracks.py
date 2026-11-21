
import pandas as pd

# ==========================================
# SIMULTANEOUS TRACK VALIDATION
# ==========================================

INPUT_FILE = "radar_association_log_frame.csv"
OUTPUT_FILE = "simultaneous_track_details.csv"

# Read association log
try:
    df = pd.read_csv(INPUT_FILE)

except FileNotFoundError:
    print("ERROR: Radar association log not found.")
    print("Check that you are running this script from the main project folder.")
    raise SystemExit(1)


# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()


# Convert columns to appropriate data types
df["Timestamp"] = pd.to_numeric(
    df["Timestamp"], errors="coerce"
)

df["Sensor_ID"] = pd.to_numeric(
    df["Sensor_ID"], errors="coerce"
)

df["Internal_ID"] = pd.to_numeric(
    df["Internal_ID"], errors="coerce"
)

df = df.dropna(
    subset=[
        "Timestamp",
        "Sensor_ID",
        "Internal_ID",
        "Reference_ID"
    ]
)


# Group by sensor, reference object and exact timestamp
grouped = (
    df.groupby(
        ["Sensor_ID", "Reference_ID", "Timestamp"]
    )
    .agg(
        Internal_ID_Count=("Internal_ID", "nunique"),
        Internal_IDs=(
            "Internal_ID",
            lambda values: sorted(values.unique().tolist())
        ),
        Detection_Count=("Internal_ID", "size")
    )
    .reset_index()
)


# Find observations with multiple internal IDs
simultaneous = grouped[
    grouped["Internal_ID_Count"] > 1
].copy()


# ==========================================
# DISPLAY RESULTS
# ==========================================

print("\n==========================================")
print("      SIMULTANEOUS TRACK VALIDATION")
print("==========================================")

print("Total labelled detections:", len(df))

print(
    "Simultaneous multi-ID observations:",
    len(simultaneous)
)


if not simultaneous.empty:

    print("\nFIRST 20 CASES")

    print(
        simultaneous.head(20).to_string(index=False)
    )

    # Save detailed results
    simultaneous.to_csv(
        OUTPUT_FILE,
        index=False
    )

    print("\nSaved:", OUTPUT_FILE)

else:

    print("\nNo simultaneous multi-ID observations found.")

    # Save an empty report with headers
    simultaneous.to_csv(
        OUTPUT_FILE,
        index=False
    )

    print("Empty validation report saved:", OUTPUT_FILE)


# ==========================================
# SENSOR-WISE ANALYSIS
# ==========================================

print("\nSIMULTANEOUS OBSERVATIONS BY SENSOR")

if not simultaneous.empty:

    print(
        simultaneous.groupby("Sensor_ID").size()
    )

else:

    print("No cases found.")


print("\n==========================================")
print("Validation completed successfully.")
print("==========================================")
