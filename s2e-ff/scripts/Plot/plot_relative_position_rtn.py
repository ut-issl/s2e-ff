#
# Plot Satellite Relative Position on RTN frame
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# local function
from common import find_latest_log_tag
# csv read
import pandas
# arguments
import argparse

#
# Read Arguments
#
aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/logs"', default='../../data/logs')
aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
aparser.add_argument('--no-gui', action='store_true')

args = aparser.parse_args()

# log file path
path_to_logs = args.logs_dir

read_file_tag = args.file_tag
if read_file_tag == None:
  print("file tag does not found. use latest the latest log file.")
  read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

#
# CSV file name
#
read_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
d1 = pandas.read_csv(read_file_name, skiprows=[1,1], sep=',', usecols=['sat1 pos from sat0_rtn_x[m]', 'sat1 pos from sat0_rtn_y[m]', 'sat1 pos from sat0_rtn_z[m]'])
# Add satellites if you need
# d2 = pandas.read_csv(read_file_name, skiprows=[1,1], sep=',', usecols=['sat2 pos from sat0_rtn_x[m]', 'sat2 pos from sat0_rtn_y[m]', 'sat2 pos from sat0_rtn_z[m]'])

# Edit data if you need

#
# Plot
#
fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Relative Position of Satellites in RTN frame")
ax.set_xlabel("Radial [m]")
ax.set_ylabel("Transverse [m]")
ax.set_zlabel("Normal [m]")

# Add plot settings if you need
#ax.set_xlim(-100, 100)
#ax.set_ylim(-100, 100)
#ax.set_zlim(-100, 100)

ax.plot(0,0,0, marker="*", c="green", markersize=10, label="Sat0")
ax.plot(d1['sat1 pos from sat0_rtn_x[m]'].to_numpy(),d1['sat1 pos from sat0_rtn_y[m]'].to_numpy(),d1['sat1 pos from sat0_rtn_z[m]'].to_numpy(), marker="x", c="red", label="Sat1")
# Add satellites if you need
# ax.plot(d2['sat2 pos from sat0_rtn_x[m]'].to_numpy(),d2['sat2 pos from sat0_rtn_y[m]'].to_numpy(),d2['sat2 pos from sat0_rtn_z[m]'].to_numpy(), marker="o", c="blue", label="Sat2")

ax.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_relative_position_rtn.png")
else:
  plt.show()
