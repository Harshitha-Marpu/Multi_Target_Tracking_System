
import pandas as pd
import numpy as np

df = pd.read_csv("radar_association_log_frame.csv")

df = df[
    (df["Sensor_ID"] == 3)
    & (
        df["Reference_ID"]
        == "f6a76074746011e8bfed38d547008930"
    )
    & (df["Internal_ID"].isin([1063, 1212]))
].copy()

for col in ["Timestamp", "Internal_ID", "X", "Y"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.dropna(
    subset=["Timestamp", "Internal_ID", "X", "Y"]
)

# Keep only the overlapping time interval
df = df[
    (df["Timestamp"] >= 57.825)
    & (df["Timestamp"] <= 58.641)
]

print("\n========== OVERLAPPING TRACK DETAILS ==========")

for internal_id, group in df.groupby("Internal_ID"):

    print(f"\nINTERNAL ID: {int(internal_id)}")

    print("Detection count:", len(group))

    print("First timestamp:", group["Timestamp"].min())

    print("Last timestamp:", group["Timestamp"].max())

    print("Mean X:", round(group["X"].mean(), 3))

    print("Mean Y:", round(group["Y"].mean(), 3))

    print("\nDetection positions:")

    print(
        group[
            ["Timestamp", "X", "Y"]
        ].to_string(index=False)
    )

# Compare mean positions
summary = df.groupby("Internal_ID")[["X", "Y"]].mean()

if len(summary) == 2:

    distance = np.sqrt(
        (summary.iloc[0]["X"] - summary.iloc[1]["X"]) ** 2
        + (summary.iloc[0]["Y"] - summary.iloc[1]["Y"]) ** 2
    )

    print("\nDistance between mean positions:", round(distance, 3), "metres")

print("\n==============================================")
