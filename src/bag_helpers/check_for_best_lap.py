import os
import rosbag
from alive_progress import alive_bar
import fire
import json

SCALAR_TOPIC = "/dyn_sector_server/parameter_updates"
LAP_TOPIC = "/lap_data"
from utils.files import get_text_of_last_dir


def update_candidates(scalar, lap_time, avg_e, max_e, bag, start_time, end_time, candidates, all):
    if not all:
        for c in candidates:
            if c["scalar"] == scalar:
                if c["lap_time"] > lap_time:
                    c["lap_time"] = lap_time
                    c["lat_error"] = avg_e
                    c["max_lat_error"] = max_e
                    c["bag_file"] = bag
                    c["start_time"] = start_time
                    c["end_time"] = end_time
                return candidates
    candidates.append({
        "scalar": scalar, 
        "lap_time": lap_time, 
        "lat_error": avg_e, 
        "max_lat_error": max_e, 
        "bag_file": bag,
        "start_time": start_time,
        "end_time": end_time
        })
    return candidates


def check_topics(topics, bag):
    if SCALAR_TOPIC not in topics:
        print(f"The scalar topic does not exist in the bag: {bag}. Exiting.")
        return False
    if LAP_TOPIC not in topics:
        print(f"The lap topic does not exist in the bag: {bag}. Exiting.")
        return False
    return True

def check_bag(bag, candidates, all):
    print("Checking bag: ", bag)
    with rosbag.Bag(bag, 'r') as inbag:
        if not check_topics(inbag.get_type_and_topic_info().topics.keys(), bag):
            return candidates
        n = inbag.get_message_count()
        scalar_updated = False
        scalar = 0.7
        start_time = None
        with alive_bar(n) as bar:
            for topic, msg, t in inbag.read_messages():
                bar()
                if topic == SCALAR_TOPIC:
                    new_scalar = 100
                    for d in msg.doubles:
                            new_scalar = d.value if d.value < new_scalar else new_scalar
                    if new_scalar != 100:
                        scalar = new_scalar
                        scalar_updated = True
                elif topic == LAP_TOPIC:
                        if not scalar_updated:
                            print(f"Valid Lap with scalar: {scalar} and lap time: {msg.lap_time}")
                            candidates = update_candidates(
                                scalar, 
                                msg.lap_time, 
                                msg.average_lateral_error_to_global_waypoints, 
                                msg.max_lateral_error_to_global_waypoints, 
                                bag,
                                start_time,
                                t.to_sec(),
                                candidates,
                                all)
                        scalar_updated = False
                        start_time = t.to_sec()

    return candidates

def get_cache(dir):
    cache = {}
    if os.path.exists(f"cache/check_best_lap_{dir}.txt"):
        with open(f"cache/check_best_lap_{dir}.txt", "r") as f:
            for line in f:
                lines = line.split(",")
                if len(lines) == 2:
                    cache[lines[0]] = lines[1].strip('\n')
    return cache

def write_cache(dir, cache):
    with open(f"cache/check_best_lap_{dir}.txt", "w") as f:
        for k, v in cache.items():
            f.write(f"{k},{v}\n")

def write_best_lap(candidates, dir):
    json_obj = {"candidates": candidates}
    path = f"plots/check_laps/{dir}_best_lap.json"
    
    with open(path, "w") as f:
        f.write(json.dumps(json_obj, indent=4))


def check_dir(dir, use_cache=False, all=False):
    print("Checking directory: ", dir)
    candidates = []
    cache = {}
    dir_text = get_text_of_last_dir(dir)

    if use_cache:
        cache = get_cache(dir_text)
            
    for file in os.listdir(dir):
        if file.endswith(".bag"):
            if file in cache and cache[file] == "no":
                print("Skipping bag: ", file)
                continue
            old = len(candidates)
            candidates = check_bag(os.path.join(dir, file), candidates, all)
            if file not in cache and len(candidates) == old:
                print("No valid lap found in bag: ", file)
                cache[file] = "no"

    candidates.sort(key=lambda x: x["lap_time"])
    write_cache(dir_text, cache)
    write_best_lap(candidates, dir_text)
        

if __name__ == "__main__":
    fire.Fire(check_dir)
