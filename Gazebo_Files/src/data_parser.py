import csv
import argparse
import os
# from circle_test import circle_IK
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# get cwd
path = os.getcwd()

# Argument parser:
parser = argparse.ArgumentParser()
parser.add_argument("-readfile", help="Select rosbag file", type=str)
parser.set_defaults(readfile='move_robot/_slash_data_recorder.csv')
parser.add_argument("-writefile", help="Select file to write into", type=str)
parser.set_defaults(writefile='move_robot/result.csv')

args = parser.parse_args()

data = {'time':[], 'j1_position':[], 'j1_velocity':[], 'j1_effort':[],
                    'j2_position':[], 'j2_velocity':[], 'j2_effort':[],
                    'ee_position_x':[], 'ee_position_y':[], 'ee_position_z':[] }

with open(args.readfile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            headings = row
            line_count += 1
        else:
            row = row[4].split(',')
            data['time'].append(float(row[0][1:]))
            data['j1_position'].append(float(row[1]))
            data['j1_velocity'].append(float(row[2]))
            data['j1_effort'].append(float(row[3]))
            data['j2_position'].append(float(row[4]))
            data['j2_velocity'].append(float(row[5]))
            data['j2_effort'].append(float(row[6]))
            data['ee_position_x'].append(float(row[7]))
            data['ee_position_y'].append(float(row[8]))
            data['ee_position_z'].append(float(row[9][:-1]))
    

with open(args.writefile, 'w') as write_file:
    csv_writer = csv.DictWriter(write_file, fieldnames=data.keys())
    csv_writer.writeheader()

    n_entries = len(data['time'])
    n_fieldnames = len(data.keys())
    info = {}

    for i in range(n_entries):
        for j in range(n_fieldnames):
            info[data.keys()[j]] = data[data.keys()[j]][i]

        csv_writer.writerow(info)



# Plotter
fig, axes = plt.subplots(2,2)  
(ax1, ax2), (ax3, ax4) = axes

# ax1: plot joint positions
ax1.plot(data['time'], data['j1_position'], label='Joint 1', color='r')
ax1.plot(data['time'], data['j2_position'], label='Joint 2', color='b')
ax1.set(xlabel='Time (s)', ylabel='Joint Position (rad)')
ax1.legend(loc='best')
ax1.grid()

# ax2: plot joint velocities
ax2.plot(data['time'], data['j1_velocity'], label='Joint 1', color='r')
ax2.plot(data['time'], data['j2_velocity'], label='Joint 2', color='b')
ax2.set(xlabel='Time (s)', ylabel='Joint Velocity (rad/s)')
ax2.legend(loc='best')
ax2.grid()

# ax3: plot joint efforts
ax3.plot(data['time'], data['j1_effort'], label='Joint 1', color='r')
ax3.plot(data['time'], data['j2_effort'], label='Joint 2', color='b')
ax3.set(xlabel='Time (s)', ylabel='Joint Effort (N-m)')
ax3.legend(loc='best')
ax3.grid()

# ax4: plot ee_position
ax4.plot(data['ee_position_x'], data['ee_position_z'], color='b')
# add desired circle
pch = patches.Circle((-0.2, 0.8), radius=0.1, ec='r', fill=False)
ax4.add_patch(pch)

ax4.set(xlabel='x-axis', ylabel='z-axis')
ax4.set_xlim(-0.5, 0.5)
ax4.set_ylim(0.5, 1.5)
ax4.grid()

plt.show()