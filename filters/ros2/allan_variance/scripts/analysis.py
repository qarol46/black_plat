#!/usr/bin/env python3

"""
@file   analysis.py
@brief  Plotting and analysis tool to determine IMU parameters.
@author Russell Buchanan
"""

import argparse
import csv
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.ticker as ticker

def line_func(x, m, b):
	return m * x + b

def get_intercept(x, y, m, b):
	logx = np.log(x)
	logy = np.log(y)
	coeffs, _ = curve_fit(line_func, logx, logy, bounds=([m, -np.inf], [m + 0.001, np.inf]))
	poly = np.poly1d(coeffs)
	yfit = lambda x: np.exp(poly(np.log(x)))
	return yfit(b), yfit


def generate_prediction(tau, q_quantization=0, q_white=0, q_bias_instability=0, q_walk=0, q_ramp=0):
	n = len(tau)
	A = np.empty((n, 5))
	A[:, 0] = 3 / tau**2
	A[:, 1] = 1 / tau
	A[:, 2] = 2 * np.log(2) / np.pi
	A[:, 3] = tau / 3
	A[:, 4] = tau**2 / 2
	params = np.array([q_quantization ** 2, q_white ** 2, q_bias_instability ** 2, q_walk ** 2, q_ramp ** 2])
	return np.sqrt(A.dot(params))

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--data', metavar='STR', type=str, help='TUM data files to plot')
parser.add_argument('--config', metavar='STR', type=str, help='yaml config file')
parser.add_argument("--skip", type=int, default=1)
parser.add_argument("--output", type=str, default="imu.yaml")
args = parser.parse_args()

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.weight": "bold",
    "axes.labelweight": "bold",
    "axes.titleweight": "bold",
    "text.latex.preamble": r"""
        \usepackage[T2A]{fontenc}
        \usepackage[utf8]{inputenc}
        \usepackage[russian]{babel}
        \usepackage{mathptmx}
        \renewcommand{\seriesdefault}{b}
        \boldmath
    """,
})

# Load config file if provided
rostopic = "/imu/data"
update_rate = 400.0
if args.config:
	import yaml
	with open(args.config, "r") as stream:
		config = yaml.safe_load(stream)
	rostopic = config["imu_topic"]
	update_rate = config["imu_rate"]

# Initialize arrays
period = np.array([])
acceleration = np.empty((0,3), float)
rotation_rate = np.empty((0,3), float)

# Read data from file
with open(args.data) as input_file:
	csv_reader = csv.reader(input_file, delimiter=' ')
	counter = 0
	for row in csv_reader:
		counter = counter + 1
		if (counter % args.skip != 0):
			continue
		t = float(row[0])
		period = np.append(period, [t], axis=0)
		acceleration = np.append(acceleration, np.array([float(row[1]), float(row[2]), float(row[3])]).reshape(1,3), axis=0)
		rotation_rate = np.append(rotation_rate, np.array([float(row[4]), float(row[5]), float(row[6])]).reshape(1,3), axis=0)

# -----------------------------------------------------------------------
# Accelerometer
# -----------------------------------------------------------------------
white_noise_break_point = np.where(period == 10)[0][0]
random_rate_break_point = np.where(period == 10)[0][0]

accel_wn_intercept_x, xfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,0], -0.5, 1.0)
accel_wn_intercept_y, yfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,1], -0.5, 1.0)
accel_wn_intercept_z, zfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,2], -0.5, 1.0)

accel_rr_intercept_x, xfit_rr = get_intercept(period, acceleration[:,0], 0.5, 3.0)
accel_rr_intercept_y, yfit_rr = get_intercept(period, acceleration[:,1], 0.5, 3.0)
accel_rr_intercept_z, zfit_rr = get_intercept(period, acceleration[:,2], 0.5, 3.0)

accel_min_x = np.amin(acceleration[:,0])
accel_min_y = np.amin(acceleration[:,1])
accel_min_z = np.amin(acceleration[:,2])

accel_min_x_index = np.argmin(acceleration[:,0])
accel_min_y_index = np.argmin(acceleration[:,1])
accel_min_z_index = np.argmin(acceleration[:,2])

worst_accel_white_noise = np.amax([accel_wn_intercept_x, accel_wn_intercept_y, accel_wn_intercept_z])
worst_accel_random_walk = np.amax([accel_rr_intercept_x, accel_rr_intercept_y, accel_rr_intercept_z])

# Write to yaml file
yaml_file = open(args.output, "w")
yaml_file.write("#Accelerometer\n")
yaml_file.write("accelerometer_noise_density: " + repr(worst_accel_white_noise) + " \n")
yaml_file.write("accelerometer_random_walk: " + repr(worst_accel_random_walk) + " \n")
yaml_file.write("\n")
yaml_file.write(f"X Velocity Random Walk: {accel_wn_intercept_x} m/s/sqrt(s) {accel_wn_intercept_x*60} m/s/sqrt(hr)\n")
yaml_file.write(f"Y Velocity Random Walk: {accel_wn_intercept_y} m/s/sqrt(s) {accel_wn_intercept_y*60} m/s/sqrt(hr)\n")
yaml_file.write(f"Z Velocity Random Walk: {accel_wn_intercept_z} m/s/sqrt(s) {accel_wn_intercept_z*60} m/s/sqrt(hr)\n")
yaml_file.write(f"X Bias Instability: {accel_min_x} m/s^2 {accel_min_x*3600*3600} m/hr^2\n")
yaml_file.write(f"Y Bias Instability: {accel_min_y} m/s^2 {accel_min_y*3600*3600} m/hr^2\n")
yaml_file.write(f"Z Bias Instability: {accel_min_z} m/s^2 {accel_min_z*3600*3600} m/hr^2\n")
yaml_file.write(f"X Accel Random Walk: {accel_rr_intercept_x} m/s^2/sqrt(s)\n")
yaml_file.write(f"Y Accel Random Walk: {accel_rr_intercept_y} m/s^2/sqrt(s)\n")
yaml_file.write(f"Z Accel Random Walk: {accel_rr_intercept_z} m/s^2/sqrt(s)\n")
yaml_file.write("\n")

average_acc_white_noise = (accel_wn_intercept_x + accel_wn_intercept_y + accel_wn_intercept_z) / 3
average_acc_bias_instability = (accel_min_x + accel_min_y + accel_min_z) / 3
average_acc_random_walk = (accel_rr_intercept_x + accel_rr_intercept_y + accel_rr_intercept_z) / 3

# Plot accelerometer data

matplotlib.rcParams.update({
    'font.size': 50,          # базовый размер
    'axes.titlesize': 50,     # заголовок осей
    'axes.labelsize': 50,     # подписи осей
    'legend.fontsize': 33,    # легенда
    'xtick.labelsize': 48,    # метки X
    'ytick.labelsize': 48,    # метки Y
})
plt.rcParams["font.family"] = "Times New Roman"
dpi = 90
figsize = (16, 9)
fig1 = plt.figure(num=r"\textbf{Акселерометр}", dpi=dpi, figsize=figsize)

plt.loglog(period, acceleration[:,0], "r--")
plt.loglog(period, acceleration[:,1], "g--")
plt.loglog(period, acceleration[:,2], "b--")

plt.loglog(period, xfit_wn(period), "m-")
plt.loglog(period, yfit_wn(period), "m-")
plt.loglog(period, zfit_wn(period), "m-")

plt.loglog(period, xfit_rr(period), "y-")
plt.loglog(period, yfit_rr(period), "y-")
plt.loglog(period, zfit_rr(period), "y-")

plt.loglog(1.0, accel_wn_intercept_x, "ro", markersize=20)
plt.loglog(1.0, accel_wn_intercept_y, "go", markersize=20)
plt.loglog(1.0, accel_wn_intercept_z, "bo", markersize=20)

plt.loglog(3.0, accel_rr_intercept_x, "r*", markersize=20)
plt.loglog(3.0, accel_rr_intercept_y, "g*", markersize=20)
plt.loglog(3.0, accel_rr_intercept_z, "b*", markersize=20)

plt.loglog(period[accel_min_x_index], accel_min_x, "r^", markersize=20)
plt.loglog(period[accel_min_y_index], accel_min_y, "g^", markersize=20)
plt.loglog(period[accel_min_z_index], accel_min_z, "b^", markersize=20)

# --- НАЧАЛО БЛОКА ОФОРМЛЕНИЯ ОСЕЙ ПО ГОСТ ---
ax = plt.gca()

# 1. Включаем главную и минорную сетку
ax.grid(True, which='major', linestyle='-', linewidth=1.5, color='darkgray')
ax.grid(True, which='minor', linestyle='-', linewidth=0.5, color='lightgray')

# 2. Настраиваем логарифмический формат подписей (вид 10^x)
ax.xaxis.set_major_formatter(ticker.LogFormatterMathtext())
ax.yaxis.set_major_formatter(ticker.LogFormatterMathtext())

# 3. Настраиваем штрихи (направление внутрь графика)
ax.tick_params(axis='both', which='major', length=12, width=1.5, direction='in', pad=18, bottom=True, top=True, left=True, right=True)
ax.tick_params(axis='both', which='minor', length=6, width=1.0, direction='in', bottom=True, top=True, left=True, right=True)

ax.tick_params(axis='y', which='major', length=12, width=1.5, direction='in', pad=5, left=True, right=True)
ax.tick_params(axis='y', which='minor', length=6, width=1.0, direction='in', pad=5, left=True, right=True)
# --- КОНЕЦ БЛОКА ---

# Построение легенды
line_x = mlines.Line2D([], [], color='r', linestyle='--', label='X')
line_y = mlines.Line2D([], [], color='g', linestyle='--', label='Y')
line_z = mlines.Line2D([], [], color='b', linestyle='--', label='Z')
line_wn = mlines.Line2D([], [], color='m', linestyle='-',
                         label=r"\textbf{Прямая, соответствующая случайному блужданию скорости}")
line_rw = mlines.Line2D([], [], color='y', linestyle='-',
                         label=r"\textbf{Прямая, соответствующая случайному блужданию ускорения}")
marker_vrw = mlines.Line2D([], [], color='gray', marker='o',
                            linestyle='None', markersize=12,
                            label=r"\textbf{Случайное блуждание скорости ($\tau = 1 с$)}")
marker_arw = mlines.Line2D([], [], color='gray', marker='*',
                            linestyle='None', markersize=12,
                            label=r"\textbf{Случайное блуждание ускорения ($\tau = 3 с$)}")
marker_bias = mlines.Line2D([], [], color='gray', marker='^',
                             linestyle='None', markersize=12,
							label=r"\textbf{Смещение нуля (минимум кривой)}")

plt.title(r"\textbf{Акселерометр}")
plt.ylabel(r"\textbf{Отклонение Аллана $\log\,\sigma_A(\tau)$, м/с$^2$}", labelpad = 15)
plt.xlabel(r"\textbf{Период $\log\,\tau$, с}")
plt.grid(True)
plt.legend(
    handles=[line_x, line_y, line_z,
             line_wn, line_rw,
             marker_vrw, marker_arw, marker_bias],
    loc='lower right'
)
plt.subplots_adjust(left=0.084, bottom=0.11, right=0.99, top=0.95, wspace=0.0, hspace=0.0)

fig1.savefig('acceleration.png', dpi=600, bbox_inches="tight", pad_inches=0.05)
plt.show()

# -----------------------------------------------------------------------
# Gyroscope
# -----------------------------------------------------------------------
gyro_wn_intercept_x, xfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,0], -0.5, 1.0)
gyro_wn_intercept_y, yfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,1], -0.5, 1.0)
gyro_wn_intercept_z, zfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,2], -0.5, 1.0)

gyro_rr_intercept_x, xfit_rr = get_intercept(period, rotation_rate[:,0], 0.5, 3.0)
gyro_rr_intercept_y, yfit_rr = get_intercept(period, rotation_rate[:,1], 0.5, 3.0)
gyro_rr_intercept_z, zfit_rr = get_intercept(period, rotation_rate[:,2], 0.5, 3.0)

gyro_min_x = np.amin(rotation_rate[:,0])
gyro_min_y = np.amin(rotation_rate[:,1])
gyro_min_z = np.amin(rotation_rate[:,2])

gyro_min_x_index = np.argmin(rotation_rate[:,0])
gyro_min_y_index = np.argmin(rotation_rate[:,1])
gyro_min_z_index = np.argmin(rotation_rate[:,2])

worst_gyro_white_noise = np.amax([gyro_wn_intercept_x, gyro_wn_intercept_y, gyro_wn_intercept_z])
worst_gyro_random_walk = np.amax([gyro_rr_intercept_x, gyro_rr_intercept_y, gyro_rr_intercept_z])

# Write gyroscope parameters to yaml file
yaml_file.write("#Gyroscope\n")
yaml_file.write("gyroscope_noise_density: " + repr(worst_gyro_white_noise * np.pi / 180) + " \n")
yaml_file.write("gyroscope_random_walk: " + repr(worst_gyro_random_walk * np.pi / 180) + " \n")
yaml_file.write("\n")
yaml_file.write(f"X Angle Random Walk: {gyro_wn_intercept_x: .5f} deg/sqrt(s) {gyro_wn_intercept_x * 60: .5f} deg/sqrt(hr)\n")
yaml_file.write(f"Y Angle Random Walk: {gyro_wn_intercept_y: .5f} deg/sqrt(s) {gyro_wn_intercept_y * 60: .5f} deg/sqrt(hr)\n")
yaml_file.write(f"Z Angle Random Walk: {gyro_wn_intercept_z: .5f} deg/sqrt(s) {gyro_wn_intercept_z * 60: .5f} deg/sqrt(hr)\n")
yaml_file.write(f"X Bias Instability: {gyro_min_x: .5f} deg/s {gyro_min_x*60*60: .5f} deg/hr\n")
yaml_file.write(f"Y Bias Instability: {gyro_min_y: .5f} deg/s {gyro_min_y*60*60: .5f} deg/hr\n")
yaml_file.write(f"Z Bias Instability: {gyro_min_z: .5f} deg/s {gyro_min_z*60*60: .5f} deg/hr\n")
yaml_file.write(f"X Rate Random Walk: {gyro_rr_intercept_x: .5f} deg/s/sqrt(s)\n")
yaml_file.write(f"Y Rate Random Walk: {gyro_rr_intercept_y: .5f} deg/s/sqrt(s)\n")
yaml_file.write(f"Z Rate Random Walk: {gyro_rr_intercept_z: .5f} deg/s/sqrt(s)\n")
yaml_file.write("\n")

average_gyro_white_noise = (gyro_wn_intercept_x + gyro_wn_intercept_y + gyro_wn_intercept_z) / 3
average_gyro_bias_instability = (gyro_min_x + gyro_min_y + gyro_min_z) / 3
average_gyro_random_walk = (gyro_rr_intercept_x + gyro_rr_intercept_y + gyro_rr_intercept_z) / 3

# Plot gyroscope data
fig2 = plt.figure(num=r"\textbf{Гироскоп}", dpi=dpi, figsize=figsize)

# Plot gyroscope data
RAD = np.pi / 180

plt.loglog(period, rotation_rate[:,0] * RAD, "r-")
plt.loglog(period, rotation_rate[:,1] * RAD, "g-")
plt.loglog(period, rotation_rate[:,2] * RAD, "b-")

plt.loglog(period, xfit_wn(period) * RAD, "m-")
plt.loglog(period, yfit_wn(period) * RAD, "m-")
plt.loglog(period, zfit_wn(period) * RAD, "m-")

plt.loglog(period, xfit_rr(period) * RAD, "y-")
plt.loglog(period, yfit_rr(period) * RAD, "y-")
plt.loglog(period, zfit_rr(period) * RAD, "y-")

plt.loglog(1.0, gyro_wn_intercept_x * RAD, "ro", markersize=20)
plt.loglog(1.0, gyro_wn_intercept_y * RAD, "go", markersize=20)
plt.loglog(1.0, gyro_wn_intercept_z * RAD, "bo", markersize=20)

plt.loglog(3.0, gyro_rr_intercept_x * RAD, "r*", markersize=20)
plt.loglog(3.0, gyro_rr_intercept_y * RAD, "g*", markersize=20)
plt.loglog(3.0, gyro_rr_intercept_z * RAD, "b*", markersize=20)

plt.loglog(period[gyro_min_x_index], gyro_min_x * RAD, "r^", markersize=20)
plt.loglog(period[gyro_min_y_index], gyro_min_y * RAD, "g^", markersize=20)
plt.loglog(period[gyro_min_z_index], gyro_min_z * RAD, "b^", markersize=20)

# --- НАЧАЛО БЛОКА ОФОРМЛЕНИЯ ОСЕЙ ПО ГОСТ ---
ax = plt.gca()

# 1. Включаем главную и минорную сетку
ax.grid(True, which='major', linestyle='-', linewidth=1.5, color='darkgray')
ax.grid(True, which='minor', linestyle='-', linewidth=0.5, color='lightgray')

# 2. Настраиваем логарифмический формат подписей (вид 10^x)
ax.xaxis.set_major_formatter(ticker.LogFormatterMathtext())
ax.yaxis.set_major_formatter(ticker.LogFormatterMathtext())

# 3. Настраиваем штрихи (направление внутрь графика)
ax.tick_params(axis='both', which='major', length=12, width=1.5, direction='in', pad=18, bottom=True, top=True, left=True, right=True)
ax.tick_params(axis='both', which='minor', length=6, width=1.0, direction='in', pad=-2, bottom=True, top=True, left=True, right=True)
# --- КОНЕЦ БЛОКА ---

# Построение легенды
line_x_g = mlines.Line2D([], [], color='r', linestyle='-', label='X')
line_y_g = mlines.Line2D([], [], color='g', linestyle='-', label='Y')
line_z_g = mlines.Line2D([], [], color='b', linestyle='-', label='Z')
line_wn_g = mlines.Line2D([], [], color='m', linestyle='-',
                           label='Прямая, соответствующая случайному блужданию угла')
line_rw_g = mlines.Line2D([], [], color='y', linestyle='-',
                           label='Прямая, соответствующая случайному блужданию угловой скорости')
marker_arw_g = mlines.Line2D([], [], color='gray', marker='o',
                              linestyle='None', markersize=12,
                              label='Случайное блуждание угла ($\\tau$ = 1 с)')
marker_rrw_g = mlines.Line2D([], [], color='gray', marker='*',
                              linestyle='None', markersize=12,
                              label='Случайное блуждание скорости дрейфа ($\\tau$ = 3 с)')
marker_bias_g = mlines.Line2D([], [], color='gray', marker='^',
                               linestyle='None', markersize=12,
                               label='Смещение нуля (минимум кривой)')

plt.title("Гироскоп")
plt.ylabel(r"Отклонение Аллана $\log\,\sigma_A(\tau)$, рад/с", labelpad = 15)
plt.xlabel(r"Период $\log\,\tau$, с")
plt.grid(True)
plt.legend(
    handles=[line_x_g, line_y_g, line_z_g,
             line_wn_g, line_rw_g,
             marker_arw_g, marker_rrw_g, marker_bias_g],
    loc='lower right'
)
plt.subplots_adjust(left=0.084, bottom=0.14, right=0.99, top=0.95, wspace=0.0, hspace=0.0)

fig2.savefig('gyro.png', dpi=600, bbox_inches="tight", pad_inches=0.05)
plt.show()

# Write rostopic and update rate to yaml file
if args.config:
	yaml_file.write("rostopic: " + repr(rostopic) + " \n")
	yaml_file.write("update_rate: " + repr(update_rate) + " \n")
else:
	yaml_file.write("rostopic: " + repr(rostopic) + " #Make sure this is correct\n")
	yaml_file.write("update_rate: " + repr(update_rate) + " #Make sure this is correct\n")
yaml_file.write("\n")
yaml_file.close()

print("Writing Kalibr imu.yaml file.")
print("Make sure to update the rostopic and rate in the file if a config file was not provided.")