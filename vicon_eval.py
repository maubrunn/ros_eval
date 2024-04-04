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
VICON_TOPIC = '/vesc/odom'
ODOM_TOPIC = '/car_state/odom'
PLOT = True
PATH_ROOT = '/home/moe/bagfiles/gokart/carla'
MAX_DT = 0.01

VEL_ALPHA = 0.1

BAGS = [
    'carlamap_bb.bag',
    'carlamap_fast_bb.bag',
    'wintimap_bb.bag', 
    'wintimap_herbie.bag'
]

RESULTS_PATH = "carla_eval"

overall_dt = 0


class TfBuffer:
    def __init__(self, topic_name, bag_path):
        self.bag_path = bag_path
        self.buffer = []
        self.curr_index = 0
    
        # read into_buffer
        with rosbag.Bag(bag_path) as vicon_bag:
            for _, msg, t in vicon_bag.read_messages(topics=[topic_name]):
                self.buffer.append((msg, t.to_sec()))
     

    def get_transform_at_time(self, time):
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

def evaluate_bag(bag_path):
    df = pd.DataFrame(columns=["time, position_x, position_y, velocity_x, velocity_y"])
    error_x = 0
    error_vx = 0
    error_yaw = 0
    error_y = 0
    error_vy = 0
    tf_buffer = TfBuffer(VICON_TOPIC, bag_path)
    i = 0
    with rosbag.Bag(bag_path) as bag:
        n = bag.get_message_count(ODOM_TOPIC)
        with alive_bar(n) as bar:
            last_vx_vicon = 0
            last_vy_vicon = 0
            for _, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
                vicon_msg = tf_buffer.get_transform_at_time(t.to_sec())
                vx_vicon = 0
                vy_vicon = 0
                bar()
                if vicon_msg is not None and tf_buffer.curr_index > 0:
                    # compute mean squared error
                    last_vicon_msg = tf_buffer.buffer[tf_buffer.curr_index - 1][0]
                    dt = vicon_msg.header.stamp.to_sec() - last_vicon_msg.header.stamp.to_sec() 

                    yaw = euler_from_quaternion([
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])[2]
                    
                    if 'vicon' in VICON_TOPIC:
                        vicon_pos_x = vicon_msg.transform.translation.x
                        vicon_pos_y = vicon_msg.transform.translation.y
                        vicon_yaw = euler_from_quaternion([
                                    vicon_msg.transform.rotation.x,
                                    vicon_msg.transform.rotation.y,
                                    vicon_msg.transform.rotation.z,
                                    vicon_msg.transform.rotation.w])[2]
                        vx_glob_vicon = (vicon_msg.transform.translation.x - last_vicon_msg.transform.translation.x) / dt
                        vy_glob_vicon = (vicon_msg.transform.translation.y - last_vicon_msg.transform.translation.y) / dt

                        last_vicon_yaw = euler_from_quaternion([
                                    last_vicon_msg.transform.rotation.x,
                                    last_vicon_msg.transform.rotation.y,
                                    last_vicon_msg.transform.rotation.z,
                                    last_vicon_msg.transform.rotation.w])[2]
                        if last_vicon_yaw - vicon_yaw > math.pi:
                            vicon_yaw += 2 * math.pi
                        vyaw_vicon = (vicon_yaw - last_vicon_yaw) / dt
                        vx_vicon = vx_glob_vicon * math.cos(vicon_yaw) + vy_glob_vicon * math.sin(vicon_yaw)
                        vy_vicon = -vx_glob_vicon * math.sin(vicon_yaw) + vy_glob_vicon * math.cos(vicon_yaw)
                        vx_vicon = VEL_ALPHA * vx_vicon + (1 - VEL_ALPHA) * last_vx_vicon
                        vy_vicon = VEL_ALPHA * vy_vicon + (1 - VEL_ALPHA) * last_vy_vicon
                    else:
                        vicon_yaw = euler_from_quaternion([
                                    vicon_msg.pose.pose.orientation.x,
                                    vicon_msg.pose.pose.orientation.y,
                                    vicon_msg.pose.pose.orientation.z,
                                    vicon_msg.pose.pose.orientation.w])[2]
                        vicon_pos_x = vicon_msg.pose.pose.position.x
                        vicon_pos_y = vicon_msg.pose.pose.position.y
                        vx_vicon = vicon_msg.twist.twist.linear.x
                        vy_vicon = vicon_msg.twist.twist.linear.y
                        vyaw_vicon = vicon_msg.twist.twist.angular.z
                
                  
                    

                    last_vx_vicon = vx_vicon
                    last_vy_vicon = vy_vicon

                    error_x += (vicon_pos_x - msg.pose.pose.position.x)**2
                    error_y += (vicon_pos_y - msg.pose.pose.position.y)**2
                    error_yaw += (vicon_yaw - yaw)**2
                    error_vx += (vx_vicon - msg.twist.twist.linear.x)**2
                    error_vy += (vy_vicon - msg.twist.twist.linear.y)**2
                    error_vyaw = (vyaw_vicon - msg.twist.twist.angular.z)**2
                    # add to dataframe
                    if PLOT:
                        df = pd.concat([df, pd.DataFrame({
                            "time": t.to_sec(),
                            "position_x": msg.pose.pose.position.x,
                            "vicon_position_x": vicon_pos_x,
                            "position_y": msg.pose.pose.position.y,
                            "vicon_position_y": vicon_pos_y,
                            "velocity_x": msg.twist.twist.linear.x,
                            "vicon_velocity_x": vx_vicon,
                            "velocity_y": msg.twist.twist.linear.y,
                            "vicon_velocity_y": vy_vicon,
                            "yaw": yaw,
                            "vicon_yaw": vicon_yaw,
                            "vicon_vyaw": vyaw_vicon,
                            "vyaw": msg.twist.twist.angular.z,
                        }, index=[0])], axis=0, ignore_index=True)
                    last_vicon_msg = vicon_msg
                    i+=1
    error_x = int(math.sqrt(error_x/i) * 10000) / 10000
    error_y = int(math.sqrt(error_y/i) * 10000) / 10000
    error_yaw = int(math.sqrt(error_yaw/i) * 10000) / 10000
    error_vx = int(math.sqrt(error_vx/i) * 10000) / 10000
    error_vy = int(math.sqrt(error_vy/i) * 10000) / 10000
    error_vyaw = int(math.sqrt(error_vyaw/i) * 10000) / 10000
    print('Mean Squared Error in x: ', error_x)
    print('Mean Squared Error in y: ', error_y)
    print('Mean Squared Error in vx: ', error_vx)
    print('Mean Squared Error in vy: ', error_vy)
    print('Mean Squared Error in yaw: ', error_yaw)
    print('Mean Squared Error in angular velocity: ', error_vyaw)
    print('Number of samples: ', i)
    return df, [error_x, error_y, error_vx, error_vy, error_yaw, error_vyaw]
    

def plot_data(df, path, title, x_label, y_label, col_bag, col_vicon):
    start_time = df['time'].iloc[0]
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    plt1.plot(df['time'] - start_time, df[col_bag], label='car odom')
    plt1.plot(df['time'] - start_time, df[col_vicon], label='gt')
    # plt1.title.set_text(title)
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.grid()
    plt1.legend()
    plt1.set_xticklabels([])
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def write_errors(errors, path):
    with open(path, 'w') as f:
        f.write('Mean Squared Error in x: ' + str(errors[0]) + '\n')
        f.write('Mean Squared Error in y: ' + str(errors[1]) + '\n')
        f.write('Mean Squared Error in vx: ' + str(errors[2]) + '\n')
        f.write('Mean Squared Error in vy: ' + str(errors[3]) + '\n')
        f.write('Mean Squared Error in yaw: ' + str(errors[4]) + '\n')
        f.write('Mean Squared Error in angular velocity: ' + str(errors[5]) + '\n')
        f.close()

def main():
    j =0
    dirpath = os.path.join(os.getcwd(),'plots', RESULTS_PATH)

    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        shutil.rmtree(dirpath)
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

        camera  = "without camera" if "no" in bag_name else "with camera"
        imu_type = "before" if "old" in bag_name else "after"
        fuse = "no imu fusion"
        if "ax" in bag_name:
            fuse = "ax imu fusion"
        if "ay" in bag_name:
            fuse = "ay imu fusion"  
        title   = f"{imu_type} imu fix, {fuse}, {camera}"

        if PLOT:
            plot_data(data, os.path.join(bagdir, "positionx.pdf"), f"position x [m], {title}, RMSE = {errors[0]}", 'Time [s]', 'x [m]', "position_x", "vicon_position_x")
            plot_data(data, os.path.join(bagdir, "positiony.pdf"), f"position y [m], {title}, RMSE = {errors[1]}", 'Time [s]', 'y [m]', "position_y", "vicon_position_y")
            plot_data(data, os.path.join(bagdir, "velocityx.pdf"), f"velocity x [m/s], {title}, RMSE = {errors[2]}", 'Time [s]', 'vx [m/s]', "velocity_x", "vicon_velocity_x")
            plot_data(data, os.path.join(bagdir, "velocityy.pdf"), f"velocity y [m/s], {title}, RMSE = {errors[3]}", 'Time [s]', 'vy [m/s]', "velocity_y", "vicon_velocity_y")
            plot_data(data, os.path.join(bagdir, "yaw.pdf"), f"yaw [rad], {title}, RMSE = {errors[4]}", 'Time [s]', 'yaw [rad]', "yaw", "vicon_yaw")
            plot_data(data, os.path.join(bagdir, "yawdot.pdf"), f"Angular Velocity [rad], {title}, RMSE = {errors[5]}", 'Time [s]', 'Angular velocity z [rad / s]', "vyaw", "vicon_vyaw")
            
        print('')
    print('Overall dt: ', overall_dt)

    
if __name__ == '__main__':
    main()