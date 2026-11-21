
import pandas as pd

# Load association log
df = pd.read_csv("radar_association_log_frame.csv")

# Remove unlabelled detections
df = df[
    df["Reference_ID"].notna()
    & (df["Label_ID"] != 11)
].copy()

print("\n====================================")
print("     TRACK IDENTITY VALIDATION")
print("====================================")

# Check 1: Internal ID associated with multiple references
internal_reference_counts = (
    df.groupby(["Sensor_ID", "Internal_ID"])
    ["Reference_ID"]
    .nunique()
)

merged_tracks = internal_reference_counts[
    internal_reference_counts > 1
]

print("\nCHECK 1: POSSIBLE TRACK MERGING")

print(
    "Internal tracks associated with multiple reference IDs:",
    len(merged_tracks)
)

# Check 2: Reference ID associated with multiple internal IDs
reference_internal_counts = (
    df.groupby(["Sensor_ID", "Reference_ID"])
    ["Internal_ID"]
    .nunique()
)

fragmented_tracks = reference_internal_counts[
    reference_internal_counts > 1
]

print("\nCHECK 2: TRACK FRAGMENTATION")

print(
    "Reference objects associated with multiple internal IDs:",
    len(fragmented_tracks)
)

# Check 3: Show possible merged tracks
if len(merged_tracks) > 0:

    print("\nPOSSIBLE MERGED TRACK DETAILS")

    for (sensor, internal_id) in merged_tracks.index[:20]:

        refs = df[
            (df["Sensor_ID"] == sensor)
            & (df["Internal_ID"] == internal_id)
        ]["Reference_ID"].unique()

        print(
            "Sensor:", sensor,
            "| Internal ID:", internal_id,
            "| Reference IDs:", list(refs)
        )

# Check 4: Save detailed results
merged_details = (
    df.groupby(["Sensor_ID", "Internal_ID"])
    .agg(
        Reference_Count=("Reference_ID", "nunique"),
        Reference_IDs=(
            "Reference_ID",
            lambda x: ", ".join(sorted(x.unique()))
        ),
        Detection_Count=("Reference_ID", "size")
    )
    .reset_index()
)

merged_details = merged_details[
    merged_details["Reference_Count"] > 1
]

merged_details.to_csv(
    "possible_merged_tracks.csv",
    index=False
)

print("\nSaved: possible_merged_tracks.csv")
print("\nValidation completed.")
print("====================================")
