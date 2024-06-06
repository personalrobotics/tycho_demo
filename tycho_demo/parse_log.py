import argparse
import pickle
from typing import Any, Dict, List
import numpy as np

from tycho_demo.utils import read_raw_log

def get_args():
    parser = argparse.ArgumentParser(description="Dump recorded logs into a D4RL-style dataset")
    parser.add_argument("log_paths", nargs="+", help="The log files to parse")
    parser.add_argument("out_path", help="Where to save the created dataset")
    parser.add_argument("-o", "--obs_keys", nargs="+", help="The state keys to concatenate into an obs vector, if specified")
    parser.add_argument("-a", "--act_keys", nargs="+", help="The state keys to concatenate into an action vector, if specified")
    return parser.parse_args()

def lod_to_dol(lod: List[Dict[Any, Any]]) -> Dict[Any, list]:
    """Convert list of dicts to dict of lists"""
    if len(lod) == 0:
        return {}
    combined = {k: [v] for k,v in lod[0].items()}
    for d in lod[1:]:
        assert len(d) == len(combined), "Some recordings have different keys!"
        for k, v in d.items():
            assert k in combined.keys(), "Some recordings have different keys!"
            combined[k].append(v)
    return combined

def main():
    args = get_args()

    trajs = []
    for path in args.log_paths:
        lod = list(read_raw_log(path))
        traj = lod_to_dol(lod)

        # try to stack numpy arrays where possible
        for k, v in traj.items():
            try:
                v_stacked = np.stack(v, axis=0)
                traj[k] = v_stacked
            except:
                pass

        if args.obs_keys:
            assert "observations" not in traj, "observations already in log!"
            obs = [traj[k] for k in args.obs_keys]
            obs = np.concatenate(obs, axis=-1)
            traj["observations"] = obs
        if args.act_keys:
            assert "actions" not in traj, "actions already in log!"
            act = [traj[k] for k in args.act_keys]
            act = np.concatenate(act, axis=-1)
            traj["actions"] = act

        trajs.append(traj)

    with open(args.out_path, "wb") as f:
        pickle.dump(trajs, f)

if __name__ == "__main__":
    main()
