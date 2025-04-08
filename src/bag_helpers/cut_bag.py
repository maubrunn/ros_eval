import os
import fire
import subprocess
from utils.files import get_laptimes_from_timefile
from utils.rosbag import get_states_from_bag
from sklearn.neighbors import KDTree
import sys

def find_start_and_end_time(bag_file, x, y, min_laptime=20, threshold=10):
    state_df = get_states_from_bag(bag_file)
    # print(state_df.head())
    tree = KDTree(state_df[['position_x', 'position_y']].to_numpy())
    _, indices = tree.query([[x, y]], k=20)
    start_idx = min(indices[0])
    start_time = state_df.iloc[start_idx]["time"]
    state_df = state_df.iloc[start_idx:]
    state_df = state_df.where(state_df["time"] - start_time > min_laptime).dropna()
    state_df.reset_index(inplace=True)
    tree = KDTree(state_df[['position_x', 'position_y']].to_numpy())
    _, indices = tree.query([[x, y]], k=1)
    end_idx = indices[0][0]
    end_time = state_df.iloc[end_idx]["time"]

    if state_df.iloc[end_idx]["position_x"] - x > threshold or state_df.iloc[end_idx]["position_y"] - y > threshold:
        print("End point not reached")
        return []

    print(f"0 :\tFound lap for {bag_file} with lap time {end_time - start_time}")
    return [[start_time, end_time]]



def cut_by_time(dir, file, start_time, end_time, out_dir):
    print("Cutting ", file, " from ", start_time, " to ", end_time)
    subprocess.call(["rosbag", "filter", os.path.join(dir, file), os.path.join(out_dir, file), f"t.secs >= {start_time} and t.secs <= {end_time}"])

def check_dir(dir, out_dir, time_file=None, start_x=None, start_y=None):
    print("Checking directory: ", dir)
    for file in os.listdir(dir):
        times_for_file = []
        if file.endswith(".bag"):
            print("Checking ", file)
            if time_file is not None:
                times_for_file = get_laptimes_from_timefile(os.path.join(os.getcwd(), time_file), file)
            elif start_x is not None and start_y is not None:
                try:
                    times_for_file = find_start_and_end_time(os.path.join(dir, file), start_x, start_y)
                except Exception as e:
                    print(e)
                    continue
            else:
                print("No time_file or start_x and start_y provided")
                sys.exit(1)
            if len(times_for_file) == 0:
                print("no laps found for ", file)
                continue
            try:
                idx = int(input("Choose lap to cut: "))
                cut_by_time(dir, file, times_for_file[idx][0], times_for_file[idx][1], out_dir)
            except ValueError:
                print("going to next file")
                continue
            
        

if __name__ == "__main__":
    fire.Fire(check_dir)
