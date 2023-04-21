#!/usr/bin/env python3

import argparse
from pathlib import Path
import pandas as pd
import numpy as np

from matplotlib import pyplot as plt
import seaborn as sns

from multirotors_landing_rl.plotting.plotting_utils import running_statistics


WINDOW_SIZE_DEFAULT=100

def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-fig', action='store_true',
                        help="Whether to store the output images")
    parser.add_argument('--dont-show-fig', action='store_true',
                        help="Whether to show the output images")
    parser.add_argument('--window-size', type=int, default=WINDOW_SIZE_DEFAULT,
                        help=f'Size of the window used to smooth the data (default: {WINDOW_SIZE_DEFAULT})')
    parser.add_argument('-f', '--folder', type=str, default='', required=True,
                        help='Path to folder with data')
    return parser


if __name__ == "__main__":

    # Read arguments
    args = parser().parse_args()
    logger_dir = Path(args.folder).expanduser()

    fig, ax = plt.subplots(1, 1, figsize=(7, 4))
    for subdir in logger_dir.iterdir():
        if not subdir.is_dir(): continue

        # Data
        data = pd.read_csv(subdir.joinpath("progress.csv"))

        # Statistics
        timesteps = data["time_elapsed"].dropna() / 3600.
        avgs, stds = np.zeros((data["ep_reward_mean"].dropna().shape[0])), \
                    np.zeros((data["ep_reward_mean"].dropna().shape[0]))
        for i in range(data["ep_reward_mean"].dropna().shape[0]):
            avgs[i], stds[i] = running_statistics(data["ep_reward_mean"].dropna(), i,
                                                args.window_size)

        sns.lineplot(x=timesteps, y=avgs, ax=ax, label=subdir.name)
        ax.fill_between(timesteps, (avgs - stds), (avgs + stds), alpha=.3)
        
    plt.xlabel("Time [h]")
    plt.ylabel("Reward")
    plt.legend(loc='best')

    if not args.dont_show_fig:
        plt.show()
