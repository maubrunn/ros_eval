import os
import fire
import json
import subprocess


def check_dir(dir, out_dir, time_file):
    print("Checking directory: ", dir)
    times = json.loads(open(time_file).read())
            
    for file in os.listdir(dir):
        times_for_file = []
        if file.endswith(".bag"):
            for c in times["candidates"]:
                if file == c["bag_file"].split("/")[-1]:
                    print(f"{len(times_for_file)}:\tFound lap for {file} with lap time {c['lap_time']}")
                    times_for_file.append([c["start_time"], c["end_time"]])
            
            if len(times_for_file) == 0:
                print("no laps found for ", file)
                continue

            try:
                idx = int(input("Choose lap to cut: "))
                print("Cutting ", file, " from ", times_for_file[idx][0], " to ", times_for_file[idx][1])
                subprocess.call(["rosbag", "filter", os.path.join(dir, file), os.path.join(out_dir, file), f"t.secs >= {times_for_file[idx][0]} and t.secs <= {times_for_file[idx][1]} and topic == '/car_state/odom'"])
            except:
                print("going to next file")
                continue
            
        

if __name__ == "__main__":
    fire.Fire(check_dir)
