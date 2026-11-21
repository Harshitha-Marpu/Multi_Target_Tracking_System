
import h5py
import numpy as np

# ==========================================
# RADARSCENES DATASET PATH
# ==========================================

FILE_PATH = (
    r"C:\Users\msvsj\Downloads\RadarScenes"
    r"\RadarScenes\data\sequence_151\radar_data.h5"
)


# ==========================================
# TIMESTAMP ANALYSIS
# ==========================================

def check_timestamps():

    print("\n========================================")
    print("       RADAR TIMESTAMP ANALYSIS")
    print("========================================")

    try:

        with h5py.File(FILE_PATH, "r") as file:

            radar = file["radar_data"]

            timestamps = radar["timestamp"][:]

            # Get unique timestamps
            unique_times = np.unique(timestamps)

            print("\nTotal radar detections:", len(timestamps))

            print("Unique timestamps:", len(unique_times))

            print("\nFirst timestamp:")
            print(timestamps[0])

            print("\nSecond timestamp:")
            print(timestamps[1])

            print("\nFirst 10 unique timestamps:")

            for timestamp in unique_times[:10]:
                print(timestamp)

            # Calculate timestamp differences
            differences = np.diff(unique_times[:10])

            print("\nTimestamp differences:")

            for difference in differences:
                print(difference)

            # Calculate median interval
            if len(differences) > 0:

                median_difference = np.median(differences)

                print("\nMedian timestamp difference:")
                print(median_difference)

                print("\nPossible time intervals:")

                print(
                    "If timestamps are nanoseconds:",
                    median_difference / 1e9,
                    "seconds"
                )

                print(
                    "If timestamps are microseconds:",
                    median_difference / 1e6,
                    "seconds"
                )

            print("\n========================================")
            print("       ANALYSIS COMPLETED")
            print("========================================")

    except FileNotFoundError:

        print("\nERROR: Radar dataset not found.")
        print("Check the FILE_PATH.")

    except Exception as error:

        print("\nERROR:", error)


# ==========================================
# PROGRAM ENTRY POINT
# ==========================================

if __name__ == "__main__":
    check_timestamps()
