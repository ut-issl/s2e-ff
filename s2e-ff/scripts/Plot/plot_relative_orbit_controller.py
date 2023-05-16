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
time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]', [1,3])

csv_data = pandas.read_csv(read_file_name, skiprows=[1,3], sep=',', usecols=['RelativeOrbitControllerDeputy_roe_est_-(0)[-]',
                                                                             'RelativeOrbitControllerDeputy_roe_est_-(1)[-]',
                                                                             'RelativeOrbitControllerDeputy_roe_est_-(2)[-]',
                                                                             'RelativeOrbitControllerDeputy_roe_est_-(3)[-]',
                                                                             'RelativeOrbitControllerDeputy_roe_est_-(4)[-]',
                                                                             'RelativeOrbitControllerDeputy_roe_est_-(5)[-]'])
roe_d1 = np.array([csv_data['RelativeOrbitControllerDeputy_roe_est_-(0)[-]'].to_numpy(),
                   csv_data['RelativeOrbitControllerDeputy_roe_est_-(1)[-]'].to_numpy(),
                   csv_data['RelativeOrbitControllerDeputy_roe_est_-(2)[-]'].to_numpy(),
                   csv_data['RelativeOrbitControllerDeputy_roe_est_-(3)[-]'].to_numpy(),
                   csv_data['RelativeOrbitControllerDeputy_roe_est_-(4)[-]'].to_numpy(),
                   csv_data['RelativeOrbitControllerDeputy_roe_est_-(5)[-]'].to_numpy()])

rel_pos_d1 = read_3d_vector_from_csv(read_file_name, 'satellite1_position_from_satellite0_rtn', 'm', [1,3])
rel_pos_d2 = read_3d_vector_from_csv(read_file_name, 'satellite2_position_from_satellite0_rtn', 'm', [1,3])

baseline_direction_target1_img = read_3d_vector_from_csv(read_file_name, 'RelativeOrbitAnalyzer_baseline_direction_target1_img', '-', [1,3])
baseline_direction_target2_img = read_3d_vector_from_csv(read_file_name, 'RelativeOrbitAnalyzer_baseline_direction_target2_img', '-', [1,3])

inter_sat_distance_01 = read_scalar_from_csv(read_file_name, 'RelativeOrbitAnalyzer_intersat_length_target1[m]', [1,3])
inter_sat_distance_02 = read_scalar_from_csv(read_file_name, 'RelativeOrbitAnalyzer_intersat_length_target2[m]', [1,3])
diff_inter_sat_distance = inter_sat_distance_02 - inter_sat_distance_01

#
# Plot
#
plt.figure(0)
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

plt.figure(1)
plt.plot(time[0], rel_pos_d1[0], markersize=5, linestyle='None', marker="x", c="red",   label="x_d1")
plt.plot(time[0], rel_pos_d1[1], markersize=5, linestyle='None', marker="x", c="green", label="y_d1")
plt.plot(time[0], rel_pos_d1[2], markersize=5, linestyle='None', marker="x", c="blue",  label="z_d1")
plt.plot(time[0], rel_pos_d2[0], markersize=5, linestyle='None', marker="+", c="orange",   label="x_d2")
plt.plot(time[0], rel_pos_d2[1], markersize=5, linestyle='None', marker="+", c="yellow", label="y_d2")
plt.plot(time[0], rel_pos_d2[2], markersize=5, linestyle='None', marker="+", c="black",  label="z_d2")
plt.title("Relative Position RTN")
plt.xlabel("Time [s]")
plt.ylabel("Relative Position [m]")
plt.legend()

plt.figure(2)
plt.plot(rel_pos_d1[0], rel_pos_d1[2], marker="o", c="blue")
plt.plot(rel_pos_d1[0][0], rel_pos_d1[2][0], marker="*", markersize=20, c="red", label="start")
plt.title("Relative Position RTN")
plt.xlabel("Radial direction [m]")
plt.ylabel("Normal direction [m]")
plt.legend()

plt.figure(3)
plt.plot(time[0], baseline_direction_target1_img[0], markersize=5, linestyle='None', marker="x", c="red",   label="x_d1")
plt.plot(time[0], baseline_direction_target1_img[1], markersize=5, linestyle='None', marker="x", c="green", label="y_d1")
plt.plot(time[0], baseline_direction_target1_img[2], markersize=5, linestyle='None', marker="x", c="blue",  label="z_d1")
plt.plot(time[0], baseline_direction_target2_img[0], markersize=5, linestyle='None', marker="+", c="orange",   label="x_d2")
plt.plot(time[0], baseline_direction_target2_img[1], markersize=5, linestyle='None', marker="+", c="yellow", label="y_d2")
plt.plot(time[0], baseline_direction_target2_img[2], markersize=5, linestyle='None', marker="+", c="black",  label="z_d2")
plt.title("Baseline direction in IMG frame")
plt.xlabel("Time [s]")
plt.ylabel("Direction vector [-]")
plt.legend()

plt.figure(4)
plt.plot(time[0], inter_sat_distance_01[0], markersize=5, linestyle='None', marker="x", c="red",   label="distance_0-1")
plt.plot(time[0], inter_sat_distance_02[0], markersize=5, linestyle='None', marker="x", c="green",   label="distance_0-2")
plt.plot(time[0], diff_inter_sat_distance[0], markersize=5, linestyle='None', marker="x", c="blue",   label="diff_distance")
plt.title("Inter-satellite distance [m]")
plt.xlabel("Time [s]")
plt.ylabel("Inter satellite distance")
plt.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_estimated_relative_roe_.png")
else:
  plt.show()
