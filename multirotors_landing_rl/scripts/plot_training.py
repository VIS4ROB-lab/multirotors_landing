#!/usr/bin/env python3

import argparse
from pathlib import Path
import pandas as pd

from matplotlib import pyplot as plt

from multirotors_landing_rl.plotting.plotting_utils import plot_reward


WINDOW_SIZE_DEFAULT=200

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

    # Variable names to plot (read from result files)
    var_names = ["ep_reward_mean", "policy_entropy", "value_loss", "policy_loss"]
    y_labels = ["Reward", "Policy Entropy", "Value Loss", "Policy Loss"]

    # Read data
    data = pd.read_csv(logger_dir.joinpath("progress.csv"))

    # Plotting
    for i in range(len(var_names)):
        plot_reward(data, logger_dir, varname=var_names[i],
                    ylabel=y_labels[i], window_size=args.window_size, 
                    save_fig=args.save_fig)
    
    if not args.dont_show_fig:
        plt.show()
