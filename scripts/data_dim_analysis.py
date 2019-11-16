import argparse
import os

import math

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from scipy.io import loadmat


# plots the following:
#   - data file
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--datafile', type=str, required=True, help='Log mat to use')
    # parser.add_argument('-en', '--env', type=str, required=True)
    parser.add_argument('-s', '--startstep', type=int, default=0, help='Start step (0)')
    parser.add_argument('-e', '--endstep', type=int, default=-1, help='End step (-1)')
    parser.add_argument('-sf', '--savefolder', type=str, default='', help='Folder to save plots to. optional')
    parser.add_argument('-sp', '--saveprefix', type=str, default='dataset_', help='Prefix to prepend to output plots. default is dataset_')
    parser.add_argument('-hd', '--hide', action='store_true', help='hide plots')
    parser.add_argument('-se', '--splitbyep', action='store_true', help='Split plots by episode, else all at once')
    parser.add_argument('-sq3', '--sqrt3', action='store_true', help='also plot sqrt of third dimension')

    args = parser.parse_args()

    if args.savefolder is not '' and not os.path.exists(args.savefolder):
        raise Exception("Save Directory %s does not exist!" % args.savefolder)

    dct = loadmat(args.datafile)
    obs = dct['obs']
    acs = dct['acs']
    state_dist = dct['state_dist']
    ac_dist = dct['ac_dist']
    episode_sizes = dct['episode_sizes']

    # dist = np.random.normal()

    assert args.endstep == -1 or args.startstep < args.endstep

    obs = obs[args.startstep:]
    if args.endstep != -1:
        obs = obs[:args.endstep]

    N = obs.shape[0]
    print("Total samples: %d" % N)

    assert not args.splitbyep # todo implement

    # simple plotting per dimension
    f, axs = plt.subplots(1, obs.shape[1], tight_layout=True, figsize=(4 * obs.shape[1], 6))
    f.suptitle("Rollout visualization")
    for d in range(obs.shape[1]):
        axs[d].set_title("Dim %d" % d)
        axs[d].set_xlabel("step")
        axs[d].set_ylabel("obs")
        axs[d].plot(range(N), obs[:, d], marker='x', label="raw")
        if d == 2 and args.sqrt3:
            axs[d].plot(range(N), np.sqrt(obs[:, d]), marker='x', label="sqrt")
            axs[d].legend()

    if args.savefolder != '':
        suff = "dim_plots.png"
        f.savefig(os.path.join(args.savefolder, args.saveprefix + suff))

    if not args.hide:
        print("Plotting...")
        plt.show()
        print("Done.")

