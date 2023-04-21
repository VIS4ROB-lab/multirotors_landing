#!/usr/bin/env python3

import argparse
from pathlib import Path
import numpy as np

from pandas import read_csv
import logging


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--folder', type=str, required=True,
                        help='Path to folder with weights')
    return parser


def find_best_weigths(folder: Path, verbose: bool=True) -> str:
    # Read the training info
    data_train = read_csv(folder.joinpath("progress.csv"),
                          usecols=['ep_reward_mean', 'total_timesteps'])
    # Best reward
    rewards = data_train['ep_reward_mean'].to_numpy()
    timesteps = data_train['total_timesteps'].to_numpy()
    idx = np.argmax(rewards)

    # Store path to the policy
    policy_weights = folder.joinpath("quad_land_" + str(timesteps[idx]) + "_steps.zip")
    if not policy_weights.exists():
        logging.warn("Policy weights corresponding to max reward do not exist! "\
            "Searching for closest existing weights")
        idx_tmp = idx + 1
        while not policy_weights.exists():
            policy_weights = folder.joinpath("quad_land_" + str(timesteps[idx_tmp]) + "_steps.zip")
            idx_tmp += 1

    # Print to terminal the policy we want
    if verbose:
        logging.warn(f"IDX {idx} -> max reward: {rewards[idx]} -> "
                    f"policy: {policy_weights.name.split('/')[-1]}")

    return policy_weights


if __name__ == "__main__":
    # Read arguments
    args = parser().parse_args()
    root_dir = Path(args.folder).expanduser()

    if not root_dir.exists():
        logging.error("Specify existing folder")
        exit(-1)
    elif len([f for f in root_dir.iterdir()]) == 0:
        logging.error("Input folder is empty")
        exit(-1)

    # Find best weights
    find_best_weigths(root_dir, verbose=True)
    