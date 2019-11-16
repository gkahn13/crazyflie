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
    parser.add_argument('-f', '--datafile', type=str, required=True, help='Evaluation rollout for trajectories')
    parser.add_argument('-et', '--evaltype', type=str, required=True, help='Evaluation rollout type')
    # parser.add_argument('-fbox', '--datafile_box', type=str, required=True, help='Evaluation rollout for box trajectories')
    # parser.add_argument('-fcirc', '--datafile_circle', type=str, required=True, help='Evaluation rollout for circle trajectories')
    # parser.add_argument('-fsin', '--datafile_sine', type=str, required=True, help='Evaluation rollout for sine trajectories')
    # parser.add_argument('-fcen', '--datafile_center', type=str, required=True, help='Evaluation rollout for center trajectories')
    # parser.add_argument('-en', '--env', type=str, required=True)
    parser.add_argument('-sf', '--savefolder', type=str, default='', help='Folder to save plots to. optional')
    parser.add_argument('-sp', '--saveprefix', type=str, default='dataset_', help='Prefix to prepend to output plots. default is dataset_')
    parser.add_argument('-hd', '--hide', action='store_true', help='hide plots')
    # parser.add_argument('-sq3', '--sqrt3', action='store_true', help='also plot sqrt of third dimension')

    args = parser.parse_args()

    if args.savefolder is not '' and not os.path.exists(args.savefolder):
        raise Exception("Save Directory %s does not exist!" % args.savefolder)

    dct = loadmat(args.datafile)
    # dct_box = loadmat(args.datafile_box)
    # dct_circle = loadmat(args.datafile_circle)
    # dct_sine = loadmat(args.datafile_sine)
    # dct_center = loadmat(args.datafile_center)

    evals = {
        args.evaltype : dct,
        # "circle" : dct_circle,
        # "sine" : dct_sine,
        # "center" : dct_center
    }


    # obs = dct['obs']
    # acs = dct['acs']
    # state_dist = dct['state_dist']
    # ac_dist = dct['ac_dist']
    # episode_sizes = dct['episode_sizes']

    # dist = np.random.normal()

    # assert args.endstep == -1 or args.startstep < args.endstep

    # obs = obs[args.startstep:]
    # if args.endstep != -1:
    #     obs = obs[:args.endstep]

    # N = obs.shape[0]
    # print("Total samples: %d" % N)

    # assert not args.splitbyep # todo implement

    for eval_key in evals.keys():
        data = evals[eval_key]

        episode_sizes = data['episode_sizes'][0]
        failed_ep = data['failed_ep'][:, 0]

        # simple plotting per dimension
        f, axs = plt.subplots(1, 3, tight_layout=True, figsize=(16, 6))
        f.suptitle("Evaluations for %s" % eval_key)

        # things to plot for each eval task:

        # 1. histogram of total rewards & average/std
        ep_rew = data['ep_rewards'][:, 1] # second entry reflects reward from task start (rather than total)
        axs[0].bar(range(len(ep_rew)), ep_rew)
        failed_idxs = np.where(failed_ep != 0)[0]
        axs[0].bar(failed_idxs, ep_rew[failed_idxs])
        axs[0].set_xlabel("episode")
        axs[0].set_ylabel("rewards")
        axs[0].set_title("Success Episode Rewards (Avg = %f)" % np.mean(ep_rew[~failed_ep.astype(bool)]))

        # 2. plotting rewards for each trajectory
        all_rew_list = np.split(data['rewards'][:, 1], np.cumsum(episode_sizes)[:-1], axis=0)

        starting_points = [0] * len(all_rew_list)
        nonzero_pts_list = []

        for i in range(len(all_rew_list)):
            # import    ipdb; ipdb.set_trace()
            starting_points[i] = np.nonzero(np.abs(all_rew_list[i]) > 1e-8)[0][0]
            assert len(np.nonzero(all_rew_list[i][:starting_points[i]])[0]) == 0
            nonzero_pts = all_rew_list[i][starting_points[i]:]
            nonzero_pts_list.append(nonzero_pts)
            axs[1].plot(range(len(nonzero_pts)), nonzero_pts, label="ep %d, avg %.2f" % (i, np.mean(nonzero_pts)), marker='.')

        all_data = np.concatenate(nonzero_pts_list, axis=0)
        axs[1].set_xlabel("time step")
        axs[1].set_ylabel("rew")
        axs[1].set_title("All Rewards (Avg = %f)" % np.mean(all_data))
        axs[1].legend()

        # 2. histogram of ep lengths & average/std

        ep_sizes = [len(d) for d in nonzero_pts_list]
        axs[2].bar(range(len(ep_sizes)), ep_sizes)
        axs[2].bar(failed_idxs, [ep_sizes[i] for i in failed_idxs])
        axs[2].set_xlabel("episode")
        axs[2].set_ylabel("ep sizes")
        axs[2].set_title("Episode Lengths (Avg = %f)" % np.mean(ep_sizes))

        if args.savefolder != '':
            suff = "eval_reward_%s.png" % eval_key
            f.savefig(os.path.join(args.savefolder, args.saveprefix + suff))

    if not args.hide:
        print("Plotting...")
        plt.show()
        print("Done.")

