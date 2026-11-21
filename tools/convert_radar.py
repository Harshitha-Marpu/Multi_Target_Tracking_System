
import os
import h5py
import pandas as pd


# ==========================================
# 1. RADAR DATASET LOCATION
# ==========================================

RADAR_FILE = (
    r"C:\Users\msvsj\Downloads\RadarScenes"
    r"\RadarScenes\data\sequence_151\radar_data.h5"
)


# ==========================================
# 2. OUTPUT FOLDER
# ==========================================

OUTPUT_FOLDER = os.path.join(
    "data",
    "radarscenes",
    "sequence_151"
)


# ==========================================
# 3. CLEAN DATA
# ==========================================

def clean_dataframe(df):

    for column in df.columns:

        # Convert byte strings into normal text
        df[column] = df[column].apply(
            lambda value: value.decode("utf-8")
            if isinstance(value, bytes)
            else value
        )

    return df


# ==========================================
# 4. CONVERT HDF5 DATASET TO CSV
# ==========================================

def convert_dataset(file, dataset_name, output_name):

    print("\n--------------------------------")
    print("Processing:", dataset_name)
    print("--------------------------------")

    if dataset_name not in file:

        print("Dataset not found:", dataset_name)
        return False

    # Read HDF5 records
    records = file[dataset_name][:]

    # Convert records into DataFrame
    df = pd.DataFrame.from_records(records)

    # Clean UUID and other byte-string values
    df = clean_dataframe(df)

    # Save CSV
    output_path = os.path.join(
        OUTPUT_FOLDER,
        output_name
    )

    df.to_csv(
        output_path,
        index=False,
        encoding="utf-8"
    )

    print("CSV saved successfully:", output_path)
    print("Total records:", len(df))
    print("Total columns:", len(df.columns))

    print("\nColumn names:")

    for column in df.columns:
        print("-", column)

    print("\nFirst 3 records:")
    print(df.head(3).to_string(index=False))

    return True


# ==========================================
# 5. MAIN FUNCTION
# ==========================================

def main():

    print("\n===================================")
    print(" RADARSCENES DATA CONVERTER")
    print("===================================")

    # Check whether radar file exists
    if not os.path.isfile(RADAR_FILE):

        print("\nERROR: Radar HDF5 file not found!")
        print("Check this location:")
        print(RADAR_FILE)
        return

    # Create output directory
    os.makedirs(
        OUTPUT_FOLDER,
        exist_ok=True
    )

    try:

        # Open HDF5 file
        with h5py.File(RADAR_FILE, "r") as file:

            # Convert radar detections
            radar_status = convert_dataset(
                file,
                "radar_data",
                "radar_detections.csv"
            )

            # Convert vehicle odometry
            odometry_status = convert_dataset(
                file,
                "odometry",
                "odometry.csv"
            )

        print("\n===================================")
        print(" CONVERSION SUMMARY")
        print("===================================")

        print(
            "Radar detections:",
            "SUCCESS" if radar_status else "FAILED"
        )

        print(
            "Vehicle odometry:",
            "SUCCESS" if odometry_status else "FAILED"
        )

        if radar_status and odometry_status:
            print("\nBoth CSV files created successfully!")

        print("\nOutput folder:")
        print(os.path.abspath(OUTPUT_FOLDER))

    except Exception as error:

        print("\nCONVERSION ERROR:")
        print(error)


# ==========================================
# 6. PROGRAM ENTRY POINT
# ==========================================

if __name__ == "__main__":
    main()
