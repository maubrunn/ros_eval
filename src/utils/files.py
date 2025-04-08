import os
import shutil
import sys
import json
import pandas as pd

def get_text_of_last_dir(dir):
    if dir[-1] == "/":
        return dir.split("/")[-2]
    return dir.split("/")[-1]

def get_laptimes_from_timefile(time_file, bag_file):
    if time_file.split(".")[-1] != "json":
        print("Time file should be a json file")
        raise ValueError
    print(f"Getting laptimes from {time_file} for {bag_file}")
    times = json.loads(open(time_file).read())
    times_for_file = []
    for c in times["candidates"]:
        if bag_file == c["bag_file"].split("/")[-1]:
            print(f"{len(times_for_file)}:\tFound lap for {bag_file} with lap time {c['lap_time']}")
            times_for_file.append([c["start_time"], c["end_time"]])
    return times_for_file

def read_json(json_file, skip_first_n=0):
    if json_file.split(".")[-1] != "json":
        print("File should be a json file")
        raise ValueError
    print(f"Reading {json_file}")
    latencies_json = json.loads(open(json_file).read())
    if skip_first_n > 0:
        latencies_json = latencies_json[skip_first_n:]
    return pd.DataFrame(latencies_json)


def check_and_create_dir(dirpath):
    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        print(dirpath, " exists, do you want to overwrite it? (y/[n])")
        if input() == 'y':
            shutil.rmtree(dirpath)
        else:
            print("Exiting")
            sys.exit(1)
    os.mkdir(dirpath)

def read_global_waypoints(path: str):
    with open(path, 'r') as f:
        data = json.load(f)
        return data["global_traj_wpnts_iqp"]["wpnts"]