#!/usr/bin/env python3

import numpy as np
from pathlib import Path

from matplotlib import pyplot as plt
import seaborn as sns

sns.set_style("whitegrid")


def running_statistics(x, idx, window_size=50):
    min_idx = int(max(idx - window_size/2, 0))
    max_idx = int(min(idx + window_size/2, x.shape[0]))
    return np.mean(x[min_idx:max_idx]), np.std(x[min_idx:max_idx])


def plot_reward(data, save_dir, varname, ylabel, window_size, save_fig=False):
    _, ax = plt.subplots(1, 1, figsize=(7, 4))

    # Statistics
    timesteps = data["time_elapsed"].dropna() / 3600.
    avgs, stds = np.zeros((data[varname].dropna().shape[0])), \
                 np.zeros((data[varname].dropna().shape[0]))
    for i in range(data[varname].dropna().shape[0]):
        avgs[i], stds[i] = running_statistics(data[varname].dropna(), i,
                                              window_size)

    # Plotting                                          
    sns.lineplot(x=timesteps, y=data[varname].dropna(), ax=ax, label='Raw')
    sns.lineplot(x=timesteps, y=avgs, ax=ax, label='Smoothed')
    ax.fill_between(timesteps, (avgs - stds), (avgs + stds), color='orange', alpha=.3)
    
    plt.xlabel("Time [h]")
    plt.title(ylabel)

    if save_fig:
        # Generate folder where to save images
        output_folder = Path(save_dir).joinpath("plots")
        if not output_folder.exists():
            output_folder.mkdir(parents=True)

        plt.savefig(output_folder.joinpath(ylabel + ".pdf"), dpi=600)
