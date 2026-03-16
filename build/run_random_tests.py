import json
import random
import copy
import subprocess
import os
import sys
import csv

N_RUNS = 3

INPUT_CLOUD = sys.argv[1]
CONFIG_FILE = sys.argv[2]
CONFIG_PATH = os.path.join("configs", CONFIG_FILE)

with open(CONFIG_PATH) as f:
    ranges = json.load(f)


RESULTS_CSV = "results.csv"

# create CSV with headers if it doesn't exist
if not os.path.exists(RESULTS_CSV):
    with open(RESULTS_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([])  # we'll add headers dynamically after first run

def flatten_dict(d, parent_key='', sep='_'):
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key, sep=sep))
        else:
            items[new_key] = v
    return items

def sample_value(v):

    # normal numeric range
    if isinstance(v, list) and len(v) == 2 and all(isinstance(x,(int,float)) for x in v):
        low, high = v
        if isinstance(low,int) and isinstance(high,int):
            return random.randint(low, high)
        else:
            return random.uniform(low, high)

    # range-of-range case
    if (
        isinstance(v, list)
        and len(v) == 2
        and all(isinstance(x,list) for x in v)
    ):

        min_low, min_high = v[0]
        max_low, max_high = v[1]
        min_val, max_val = None, None

        if isinstance(min_low, int) and isinstance(min_high, int) and isinstance(max_low, int) and isinstance(max_high, int):
            min_val = random.randint(min_low, min_high)
            max_val = random.randint(max_low, max_high)
        else:
            min_val = random.uniform(min_low, min_high)
            max_val = random.uniform(max_low, max_high)

        if min_val > max_val:
            min_val, max_val = max_val, min_val

        return [min_val, max_val]

    return v


def sample_config(d):

    result = {}

    for k,v in d.items():

        if isinstance(v, dict):
            result[k] = sample_config(v)

        else:
            result[k] = sample_value(v)

    return result

all_runs = []

for i in range(N_RUNS):

    config = sample_config(ranges)

    config_name = f"config_{i}.json"
    out_config_path = os.path.join('configs', config_name)

    with open(out_config_path,"w") as f:
        json.dump(config,f,indent=2)

    print("Running config", i)

    subprocess.run([
        "bash",
        "main.sh",
        INPUT_CLOUD,
        config_name
    ])

    # read your pipeline result (assuming one integer per file)
    with open("results/results.txt") as f:
        rgb, hsv, lab = [int(x.strip()) for x in f.readlines()]

    # flatten config for CSV
    flat_config = flatten_dict(config)
    flat_config["rgb_count"] = rgb
    flat_config["hsv_count"] = hsv
    flat_config["lab_count"] = lab

    all_runs.append(flat_config)

# write CSV
import pandas as pd

df = pd.DataFrame(all_runs)

# if file exists, append without writing headers
if os.path.exists(RESULTS_CSV):
    df.to_csv(RESULTS_CSV, mode='a', index=False, header=False)
else:
    df.to_csv(RESULTS_CSV, index=False)