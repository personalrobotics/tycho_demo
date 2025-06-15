import argparse
import numpy as np
import csv
import os
import pickle

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("folders", nargs="+")
    parser.add_argument("dest")
    parser.add_argument("-b", "--ball", action="store_true")
    parser.add_argument("-r", "--rigidbody", action="store_true")
    return parser.parse_args()

def parse_ndarray(s: str):
    arr = []
    for x in s.split(", "):
        arr.append(float(x.strip()))
    arr = np.array(arr)
    return arr

class BadLogError(Exception):
    pass

def parse_log(path, use_ball, use_rigidbody):
    obs = []
    act = []
    with open(path, "r") as f:
        lines = f.read().split("\n")
    for line in lines:
        if line == "":
            continue
        parts = line.split(",,")
        curr_pos = parse_ndarray(parts[0])
        ball = parse_ndarray(parts[1])
        rigidbody = parse_ndarray(parts[2])
        action = parse_ndarray(parts[3])

        if np.allclose(curr_pos, 0) or np.allclose(action, 0):
            continue
        o_arr = [curr_pos]
        if use_ball:
            if np.allclose(ball, 0):
                continue
            o_arr.append(ball)
        if use_rigidbody:
            if np.allclose(rigidbody, 0):
                continue
            o_arr.append(rigidbody)
        o = np.hstack(o_arr)
        obs.append(o)
        act.append(action)
    if len(obs) == 0:
        raise BadLogError()
    return {
        "observations": np.array(obs),
        "actions": np.array(act)
    }


def main():
    args = get_args()

    trajs = []
    for folder in args.folders:
        path = os.path.join(folder, "log.csv")
        try:
            traj = parse_log(path, args.ball, args.rigidbody)
            trajs.append(traj)
        except BadLogError:
            pass
    total_data = sum([len(t["observations"]) for t in trajs])
    print(f"Writing {total_data} data points from {len(trajs)} trajectories to {os.path.abspath(args.dest)}")
    with open(args.dest, "wb") as f:
        pickle.dump(trajs, f)

if __name__ == "__main__":
    main()
