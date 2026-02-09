import numpy as np
import sys

def read_npz(file_path):
    try:
        # Load the .npz file
        # 'with' ensures the file is closed automatically after reading
        with np.load(file_path) as data:
            
            # Get the list of all files (keys) inside the .npz
            keys = data.files
            
            print(f"\n--- Reading: {file_path} ---")
            print(f"Found {len(keys)} arrays: {keys}\n")
            
            # Loop through every key and print details
            for key in keys:
                array = data[key]
                print(f"Key: '{key}'")
                print(f"  - Shape: {array.shape}")
                print(f"  - Data Type: {array.dtype}")
                
                # Print a small preview of the data (first 3 items)
                # flatten() is used here just to make the preview readable
                print(f"  - Preview: {array.flatten()[:3]} ...\n")
                
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Usage Example:
# Replace 'data.npz' with your actual file name
if __name__ == "__main__":
    # You can pass the filename via command line: python read_npz.py myfile.npz
    if len(sys.argv) > 1:
        read_npz(sys.argv[1])
    else:
        # Or just manually set it here for testing
        filename = '/mnt/HDD1/phudh/home_robot/robot_ws/data/params.npz' 
        read_npz(filename)