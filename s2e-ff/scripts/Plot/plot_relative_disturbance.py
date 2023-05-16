#
# Plot Satellite Relative Disturbances
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
from common import read_3d_vector_from_csv
from common import read_scalar_from_csv
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
time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]', skiprows=[1,3])

# Airdrag 
csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', na_values=['air_drag_force_b_x[N]',
                                                                               'air_drag_force_b_y[N]',
                                                                               'air_drag_force_b_z[N]'])
f_air_sat0 = np.array([csv_data['air_drag_force_b_x[N]'].to_numpy(),
                       csv_data['air_drag_force_b_y[N]'].to_numpy(),
                       csv_data['air_drag_force_b_z[N]'].to_numpy()])
a_air_sat0 = f_air_sat0 / 40 # FIXME Magic number for satellite mass

f_air_sat1 = np.array([csv_data['air_drag_force_b_x[N].1'].to_numpy(),
                       csv_data['air_drag_force_b_y[N].1'].to_numpy(),
                       csv_data['air_drag_force_b_z[N].1'].to_numpy()])
a_air_sat1 = f_air_sat1 / 10 # FIXME Magic number for satellite mass

diff_a_air = a_air_sat1 - a_air_sat0

# SRP
csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', na_values=['srp_force_b_x[N]',
                                                                               'srp_force_b_y[N]',
                                                                               'srp_force_b_z[N]'])
f_srp_sat0 = np.array([csv_data['srp_force_b_x[N]'].to_numpy(),
                       csv_data['srp_force_b_y[N]'].to_numpy(),
                       csv_data['srp_force_b_z[N]'].to_numpy()])
a_srp_sat0 = f_srp_sat0 / 40 # FIXME Magic number for satellite mass

f_srp_sat1 = np.array([csv_data['srp_force_b_x[N].1'].to_numpy(),
                       csv_data['srp_force_b_y[N].1'].to_numpy(),
                       csv_data['srp_force_b_z[N].1'].to_numpy()])
a_srp_sat1 = f_srp_sat1 / 10 # FIXME Magic number for satellite mass

diff_a_srp = a_srp_sat1 - a_srp_sat0

# Total acceleration
csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', na_values=['spacecraft_acceleration_i_x[m/s2]',
                                                                               'spacecraft_acceleration_i_y[m/s2]',
                                                                               'spacecraft_acceleration_i_z[m/s2]'])
a_all_sat0 = np.array([csv_data['spacecraft_acceleration_i_x[m/s2]'].to_numpy(),
                       csv_data['spacecraft_acceleration_i_y[m/s2]'].to_numpy(),
                       csv_data['spacecraft_acceleration_i_z[m/s2]'].to_numpy()])
a_all_sat1 = np.array([csv_data['spacecraft_acceleration_i_x[m/s2].1'].to_numpy(),
                       csv_data['spacecraft_acceleration_i_y[m/s2].1'].to_numpy(),
                       csv_data['spacecraft_acceleration_i_z[m/s2].1'].to_numpy()])
diff_a_all = a_all_sat0 - a_all_sat1


#
# Plot
#
plt.figure(0)
plt.plot(time[0], a_air_sat0[0], marker="o", c="red",   label="Air_x")
plt.plot(time[0], a_air_sat0[1], marker="o", c="green", label="Air_y")
plt.plot(time[0], a_air_sat0[2], marker="o", c="blue",  label="Air_z")
plt.plot(time[0], a_srp_sat0[0], marker="o", c="yellow",   label="SRP_x")
plt.plot(time[0], a_srp_sat0[1], marker="o", c="gray", label="SRP_y")
plt.plot(time[0], a_srp_sat0[2], marker="o", c="orange",  label="SRP_z")
plt.title("Disturbances: Mother @ body")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

plt.figure(1)
plt.plot(time[0], a_air_sat1[0], marker="o", c="red",   label="Air_x")
plt.plot(time[0], a_air_sat1[1], marker="o", c="green", label="Air_y")
plt.plot(time[0], a_air_sat1[2], marker="o", c="blue",  label="Air_z")
plt.plot(time[0], a_srp_sat1[0], marker="o", c="yellow",   label="SRP_x")
plt.plot(time[0], a_srp_sat1[1], marker="o", c="gray", label="SRP_y")
plt.plot(time[0], a_srp_sat1[2], marker="o", c="orange",  label="SRP_z")
plt.title("Disturbances: Daughter @ body")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

plt.figure(2)
plt.plot(time[0], diff_a_air[0], marker="o", c="red",   label="Air_x")
plt.plot(time[0], diff_a_air[1], marker="o", c="green", label="Air_y")
plt.plot(time[0], diff_a_air[2], marker="o", c="blue",  label="Air_z")
plt.plot(time[0], diff_a_srp[0], marker="o", c="yellow",   label="SRP_x")
plt.plot(time[0], diff_a_srp[1], marker="o", c="gray", label="SRP_y")
plt.plot(time[0], diff_a_srp[2], marker="o", c="orange",  label="SRP_z")
plt.title("Disturbances: Daughter @ body")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

plt.figure(3)
plt.plot(time[0], diff_a_all [0], marker="o", c="red",   label="x")
plt.plot(time[0], diff_a_all [1], marker="o", c="green", label="y")
plt.plot(time[0], diff_a_all [2], marker="o", c="blue",  label="z")
plt.title("Difference of total disturbances @ ECI")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_relative_disturbances_.png")
else:
  plt.show()
