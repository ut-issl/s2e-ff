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
csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', usecols=['time[sec]'])
time = np.array([csv_data['time[sec]'].to_numpy()])

csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', usecols=['RelativeOrbitController_roe_est_-(0)[-]',
                                                                             'RelativeOrbitController_roe_est_-(1)[-]',
                                                                             'RelativeOrbitController_roe_est_-(2)[-]',
                                                                             'RelativeOrbitController_roe_est_-(3)[-]',
                                                                             'RelativeOrbitController_roe_est_-(4)[-]',
                                                                             'RelativeOrbitController_roe_est_-(5)[-]'])
roe_d1 = np.array([csv_data['RelativeOrbitController_roe_est_-(0)[-]'].to_numpy(),
                   csv_data['RelativeOrbitController_roe_est_-(1)[-]'].to_numpy(),
                   csv_data['RelativeOrbitController_roe_est_-(2)[-]'].to_numpy(),
                   csv_data['RelativeOrbitController_roe_est_-(3)[-]'].to_numpy(),
                   csv_data['RelativeOrbitController_roe_est_-(4)[-]'].to_numpy(),
                   csv_data['RelativeOrbitController_roe_est_-(5)[-]'].to_numpy()])

#
# Plot
#
plt.plot(time[0], roe_d1[0], marker="o", c="red",    label="da_d1")
plt.plot(time[0], roe_d1[1], marker="o", c="green",  label="dlambda_d1")
plt.plot(time[0], roe_d1[2], marker="o", c="blue",   label="dex_d1")
plt.plot(time[0], roe_d1[3], marker="o", c="yellow", label="dey_d1")
plt.plot(time[0], roe_d1[4], marker="o", c="orange", label="dix_d1")
plt.plot(time[0], roe_d1[5], marker="o", c="black",  label="diy_d1")
plt.title("Relative Orbital Elements")
plt.xlabel("Time [s]")
plt.ylabel("Estimated Relative ROE [-]")
plt.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_estimated_relative_roe_.png")
else:
  plt.show()
