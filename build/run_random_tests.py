import json
import random
import copy
import subprocess
import os

N_RUNS = 200

INPUT_CLOUD = "input.ply"
RANGE_FILE = "config_ranges.json"
CONFIG_DIR = "configs"

os.makedirs(CONFIG_DIR, exist_ok=True)

with open(RANGE_FILE) as f:
    ranges = json.load(f)


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


for i in range(N_RUNS):

    config = sample_config(ranges)

    config_path = f"{CONFIG_DIR}/config_{i}.json"

    with open(config_path,"w") as f:
        json.dump(config,f,indent=2)

    print("Running config", i)

    subprocess.run([
        "bash",
        "main.sh",
        INPUT_CLOUD,
        config_path
    ])