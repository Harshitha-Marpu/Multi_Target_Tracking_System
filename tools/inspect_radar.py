
import h5py

RADAR_FILE = (
    r"C:\Users\msvsj\Downloads\RadarScenes"
    r"\RadarScenes\data\sequence_151\radar_data.h5"
)

def inspect_file():
    with h5py.File(RADAR_FILE, "r") as file:

        print("========== HDF5 DATASET STRUCTURE ==========")

        def show_dataset(name, item):
            if isinstance(item, h5py.Dataset):
                print("\nDataset path:", name)
                print("Shape:", item.shape)
                print("Data type:", item.dtype)

                if item.dtype.names:
                    print("Fields:")

                    for field in item.dtype.names:
                        print("  ", field)

        file.visititems(show_dataset)

if __name__ == "__main__":
    inspect_file()
