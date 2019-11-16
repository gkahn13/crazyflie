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
    parser.add_argument('-od', '--obdim', type=int, required=True, help='which dimension to plot')
    parser.add_argument('-ad', '--acdim', type=int, required=True, help='which dimension to plot')
    parser.add_argument('-f', '--datafile', type=str, required=True, help='Log mat to use')
    # parser.add_argument('-en', '--env', type=str, required=True)
    parser.add_argument('-nb', type=int, required=True, help='Number of action bins to use (number of plots)')
    parser.add_argument('-ns', type=int, default=1, help='Number of steps in the future for next state')
    # parser.add_argument('-nsb', type=int, required=True, help='Number of state bins to use (number of bins in state histogram)')
    parser.add_argument('-s', '--savefolder', type=str, default='', help='Store all plots to this directory [\'\']')
    parser.add_argument('-sp', '--saveprefix', type=str, default='dataset_', help='Prefix to prepend to output plots')
    parser.add_argument('-hd', '--hide', action='store_true', help='hide plots')
    parser.add_argument('-sq', '--sqrt', action='store_true', help='sqrt the observations')

    args = parser.parse_args()

    if args.savefolder is not '' and not os.path.exists(args.savefolder):
        raise Exception("Save Directory %s does not exist!" % args.savefolder)

    dct = loadmat(args.datafile)
    obs = dct['obs']
    acs = dct['acs']
    state_dist = dct['state_dist']
    ac_dist = dct['ac_dist']
    episode_sizes = dct['episode_sizes']

    assert args.ns > 0

    # dist = np.random.normal()

    # conditional action histograms (3x3 for pendulum)

    # given current state
    obs_dim = obs[:, args.obdim]
    cur_state = obs_dim[:-args.ns]
    next_state = obs_dim[args.ns:]

    if args.sqrt:
        next_state = np.sqrt(next_state)
        cur_state = np.sqrt(cur_state)

    state_diff = next_state - cur_state

    # given action
    acs_dim = acs[:, args.acdim]
    cur_ac = acs_dim[:-args.ns]

    # chunk by actions (< 1e-6, > 1e-6, in between)
    ac_range = (np.min(cur_ac), np.max(cur_ac))
    chunk_size = (ac_range[1] - ac_range[0]) / args.nb


    start = ac_range[0]
    # histogram for each category of actions
    for i in range(args.nb):
        end = start + chunk_size

        fi, axsi = plt.subplots(3, 1, tight_layout=True, figsize=(6, 10))
        fi.suptitle("Range %s -> %s" % (str(start), str(end)))

        idxs, = np.where(np.logical_and(cur_ac >= start, cur_ac <= end))
        these_obs = cur_state[idxs]
        these_next_obs = next_state[idxs]
        these_diff = state_diff[idxs]

        sorted_idxs = np.argsort(these_obs)
        sorted_obs = these_obs[sorted_idxs]
        sorted_next_obs = these_next_obs[sorted_idxs]
        sorted_diff = these_diff[sorted_idxs]

        axsi[0].set_title('one step plot')
        axsi[0].plot(range(idxs.size), sorted_obs, color='green', label='cur')
        axsi[0].plot(range(idxs.size), sorted_next_obs, color='blue', label='next')
        axsi[0].legend()

        axsi[1].set_title('Diff plot')
        axsi[1].scatter(range(idxs.size), sorted_diff, color='red', s=1)
        axsi[1].plot(range(idxs.size), np.zeros(idxs.size), color='gray', linestyle='--')

        axsi[2].set_title('Overall diff hist')
        axsi[2].hist(sorted_diff)

        start = end

        if args.savefolder != '':
            suff = "onstep_hist_NS_%d_nb_%d_od_%d_ad_%d_i_%d.png" % (args.ns, args.nb, args.obdim, args.acdim, i)
            fi.savefig(os.path.join(args.savefolder, args.saveprefix + suff))

    if not args.hide:
        print("Plotting...")
        plt.show()
        print("Done.")

