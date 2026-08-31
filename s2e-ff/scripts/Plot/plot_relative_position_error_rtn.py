#
# Import
#
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# local function
from common import find_latest_log_tag, read_3d_vector_from_csv, read_scalar_from_csv
# csv read
import pandas
# arguments
import argparse

#
# Read Arguments
#
aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/logs"', default='../../data/logs')
aparser.add_argument('--file-tag', type=str, nargs='+', help='log file tag(s) like 220627_142946')  # Allow multiple file-tags
aparser.add_argument('--reference-file', type=str, help='reference log file tag')
aparser.add_argument('--no-gui', action='store_true')

args = aparser.parse_args()

# log file path
path_to_logs = args.logs_dir

# Check if file-tags are provided, otherwise find the latest log file
read_file_tags = args.file_tag
if read_file_tags is None:
    print("file tag does not found. Using the latest log file.")
    latest_tag = find_latest_log_tag(path_to_logs)
    read_file_tags = [latest_tag]  # Wrap it in a list to keep consistency

ref_file_tag = args.reference_file
if ref_file_tag is None:
    print("reference file tag not found. Exiting.")
    exit(1)

print("logs: " + ', '.join(read_file_tags))
print("reference log: " + ref_file_tag)

#
# CSV file name for reference
#
ref_file_name = path_to_logs + '/' + 'logs_' + ref_file_tag + '/' + ref_file_tag + '_default.csv'

#
# Data read and edit
#
# Read reference file data
ref_d1 = read_3d_vector_from_csv(ref_file_name, 'satellite1_position_from_satellite0_rtn', 'm')

# Prepare to plot for each file
all_errors = []
time = None  # Assuming time is the same for all files

# Iterate over each file-tag provided
for read_file_tag in read_file_tags:
    # Construct file path for each file-tag
    read_file_name = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'
    
    # Read S2E CSV for the current file
    d1 = read_3d_vector_from_csv(read_file_name, 'satellite1_position_from_satellite0_rtn', 'm')
    
    # Read time data (assuming time is the same for all files)
    if time is None:
        time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]')[0]
    
    # Calculate error between reference and target
    error = np.array(d1) - np.array(ref_d1)
    all_errors.append((read_file_tag, error))  # Store error along with the file tag

#
# Plot
#
fig = plt.figure(figsize=(5, 5))
ax = fig.add_subplot(111)
ax.set_title("Relative Position Error Over Time")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Error [m]")

# Plot error magnitude for each file-tag
for file_tag, error in all_errors:
    error_magnitude = np.linalg.norm(error, axis=0)
    ax.plot(time, error_magnitude, label=f"File {file_tag}")

# Add grid and legend for better readability
ax.grid(True)
ax.legend()

# Save or show the plot based on args
if args.no_gui:
    plt.savefig('_'.join(read_file_tags) + "_position_error_over_time.png")
else:
    plt.show()
