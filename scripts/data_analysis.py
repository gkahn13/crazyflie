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
    parser.add_argument('-nb', type=int, required=True, help='Number of bins for each state dim for conditional actions')
    parser.add_argument('-s', '--savefolder', type=str, default='', help='Store all plots to this directory [\'\']')
    parser.add_argument('-sp', '--saveprefix', type=str, default='closed_loop_plot_', help='Prefix to prepend to output plots')
    parser.add_argument('-hd', '--hide', action='store_true', help='hide plots')
    parser.add_argument('-sa', '--sample_sorted', action='store_true', help='sample k evenly across worst to best rollouts to visualize, otherwise use k best')

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

    # conditional action histograms (3x3 for pendulum)

    sorted_dims = np.argsort(obs, axis=0)
    bin_size = obs.shape[0] // args.nb
    obs_ranges = np.amax(obs, axis=0) - np.amin(obs, axis=0)
    bin_obs_ranges = obs_ranges / args.nb

    last_range = np.amin(obs, axis=0)

    for bn in range(args.nb):
        fi, axsi = plt.subplots(obs.shape[1], acs.shape[1], tight_layout=True, figsize=(3 * obs.shape[1], 3 * acs.shape[1]))
        next_range = last_range + bin_obs_ranges
        fi.suptitle("Range %s -> %s" % (str(last_range), str(next_range)))
        for obdim in range(obs.shape[1]):
            idx_range = sorted_dims[bn * bin_size : (bn+1) * bin_size, obdim]
            for acdim in range(acs.shape[1]):
                axsi[obdim, acdim].hist(acs[idx_range, acdim])
                axsi[obdim, acdim].hist(acs[idx_range, acdim])
                # axsi[obdim, acdim].plot(acs[:, acdim])
        last_range = next_range

    f2, axs2 = plt.subplots(1 + acs.shape[1], 1, tight_layout=True, figsize=(10,10))
    # first plot rollouts
    axs2[0].set_title("Rollout Lengths")
    axs2[0].hist(episode_sizes)
    for i in range(acs.shape[1]):
        axs2[1 + i].set_title("Action dist (dim %d)" % i)
        axs2[1 + i].hist(acs[:, i])

    if args.savefolder != '':
        suff = "k_%d.png" % (args.k)
        f.savefig(os.path.join(args.savefolder, args.saveprefix + suff))

        suff = "rollouts.png"
        f2.savefig(os.path.join(args.savefolder, args.saveprefix + suff))

    if not args.hide:
        print("Plotting...")
        plt.show()
        print("Done.")

