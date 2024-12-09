#!/usr/bin/env python3
import math
import rosbag
import matplotlib.pyplot as plt
import pandas as pd
from tf.transformations import euler_from_quaternion
import os
import shutil
from alive_progress import alive_bar

plt.rcParams.update({'font.size': 26})
PLOT = True
PATH_ROOT = '/home/moe/tmp/rosbags/carla_comparison/'
BAGS = [
    'carla_tt_70.bag',
    'tt_70.bag',
    'tt_70_sim.bag',
    'tt_carla_corrected.bag'
]

MAX_DT = 0.1

VEL_ALPHA = 0.0

RESULTS_PATH = "carla_eval"
overall_dt = 0

EST_TOPIC = '/car_state/odom'
GT_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
FIELDS = {
    "gt" : "drive.speed",
    "est" : "twist.twist.linear.x"
}


class GtBuffer:
    def __init__(self, topic_name, bag_path, gt_field):
        self.bag_path = bag_path
        self.buffer = []
        self.curr_index = 0
        self.subfields = gt_field.split('.')

        # read into_buffer
        with rosbag.Bag(bag_path) as vicon_bag:
            for _, msg, t in vicon_bag.read_messages(topics=[topic_name]):
                all_fields = msg.__getattribute__(self.subfields[0])
                for subfield in self.subfields[1:]:
                    all_fields = all_fields.__getattribute__(subfield)
                self.buffer.append((all_fields, t.to_sec()))


    def get_gt_at_time(self, time):
        global overall_dt
        if time < self.buffer[0][1] - MAX_DT:
            return None
        if time > self.buffer[-1][1] + MAX_DT:
            return None
        low_index = max(self.curr_index - 50, 0)
        high_index = min(self.curr_index + 50, len(self.buffer) - 1)
        small_buff = self.buffer[low_index : high_index]
        self.curr_index =  self.buffer.index(min(small_buff, key=lambda x: abs(x[1] - time)))
        if abs(time - self.buffer[self.curr_index][1]) > MAX_DT:
            return None
        overall_dt += abs(self.buffer[self.curr_index][1] - time)
        return self.buffer[self.curr_index][0]

def get_yaw(msg, vicon=False):
    if not vicon:
        return euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w])[2]
    else:
        return euler_from_quaternion([
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w])[2]

def evaluate_bag(bag_path):
    error = 0

    gt_buffer = GtBuffer(GT_TOPIC, bag_path, FIELDS["gt"])
    est_subfields = FIELDS['est'].split('.')
    print(est_subfields)
    i = 0
    with rosbag.Bag(bag_path) as bag:
        n = bag.get_message_count(EST_TOPIC)
        data = []
        with alive_bar(n) as bar:
            for _, msg, t in bag.read_messages(topics=[EST_TOPIC]):
                gt_msg = gt_buffer.get_gt_at_time(t.to_sec())
                bar()

                if gt_msg is  None or gt_buffer.curr_index == 0:
                    print('No GT data at time: ', t.to_sec())
                    continue

                all_fields = msg.__getattribute__(est_subfields[0])
                for subfield in est_subfields[1:]:
                    all_fields = all_fields.__getattribute__(subfield)

                est_msg = all_fields
                # compute mean squared error
                error = (gt_msg - est_msg)**2
                    # add to dataframe
                if PLOT:
                    data.append({
                        "time": t.to_sec(),
                        "GT": gt_msg,
                        "estimate": est_msg,
                    })

                i+=1

    error = int(math.sqrt(error/i) * 10000) / 10000
    print('Mean Squared Error: ', error)
    print('Number of samples: ', i)
    return  pd.DataFrame(data), [error]


def plot_data(df, path, title, x_label, y_label, col_bag, col_vicon, y_lim=None):
    start_time = df['time'].iloc[0]
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    plt1.plot(df['time'] - start_time, df[col_bag], label='vx')
    plt1.plot(df['time'] - start_time, df[col_vicon], label='command')
    # plt1.title.set_text(title)
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    if y_lim is not None:
        plt1.set_ylim(y_lim)
    plt1.grid()
    plt1.legend()
    plt1.set_xticklabels([])
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def write_errors(errors, path):
    with open(path, 'w') as f:
        f.write('Mean Squared Error: ' + str(errors[0]) + '\n')
        f.close()

def main():
    j =0
    dirpath = os.path.join(os.getcwd(),'plots', RESULTS_PATH)

    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        print(dirpath, " exists, do you want to delete it? (y/[n])")
        if input() == 'y':
            shutil.rmtree(dirpath)
        else:
                print("Exiting")
                return
    os.mkdir(dirpath)

    data = []
    for bag_name in BAGS:
        bagdir = os.path.join(dirpath, bag_name.replace('.bag', ''))
        if os.path.exists(bagdir) and os.path.isdir(bagdir):
            shutil.rmtree(bagdir)
        os.mkdir(bagdir)
        print('Evaluating: ', bag_name)
        data, errors = evaluate_bag(os.path.join(PATH_ROOT, bag_name))
        write_errors(errors, os.path.join(bagdir, "errors.txt"))
        est_field = FIELDS['est']
        if PLOT:
            plot_data(
                    data,
                    os.path.join(bagdir, f"{bag_name.replace('.bag', '')}.pdf"),
                    f"{est_field} RMSE = {errors[0]}",
                    'Time [s]',
                    est_field,
                    "estimate",
                    "GT",
                    y_lim=[1.5, 4.]
            )

        print('')
    print('Overall dt: ', overall_dt)


if __name__ == '__main__':
    main()
