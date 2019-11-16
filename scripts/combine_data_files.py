import math
import numpy as np
import rosbag
from scipy.io import savemat, loadmat
import sys
import argparse

def add_val_to_filename(file_name):
    if file_name[-4:] == ".mat":
        file_name = file_name[:-4]
    file_name += "_val.mat"
    return file_name

def make_parser():
    # parse inputs
    parser = argparse.ArgumentParser(description='script to read and combine rosbag data',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-l','--input_list', nargs='+', help='list of input mats', required=True)
    parser.add_argument('-o', '--output_file', type=str, required=True,
                        help='output file for storing mat data')
    parser.add_argument('-v', '--do_val', action='store_true', help='Merge all <input>_val.mat as well into separate file')
    return parser

def bag_and_save(args, train):

    states_list = []
    acs_list = []
    all_rewards_list = []
    latent_list = []
    ep_rewards_list = []
    time_diffs_list = []
    state_diffs_list = []
    act_state_time_diffs_list = []
    episode_sizes_list = []
    ep_end_rewards_list = []
    failed_ep_list = []
    skipped = 0

    mats = args.input_list
    if not train:
        mats = [add_val_to_filename(mat) for mat in mats]

    # iterate through all the input bag files
    for filename in mats:

        print("Reading:", filename)

        data = loadmat(filename)

        states_list.append(data['obs'])
        acs_list.append(data['acs'])
        all_rewards_list.append(data['rewards'])
        latent_list.append(data['latent'])
        ep_rewards_list.append(data['ep_rewards'])
        time_diffs_list.append(data['diffs'])
        state_diffs_list.append(data['state_diffs'])
        act_state_time_diffs_list.append(data['ac_state_time_diffs'])
        episode_sizes_list.append(data['episode_sizes'])
        ep_end_rewards_list.append(data['ep_end_rewards'])
        failed_ep_list.append(data['failed_ep'])

    # numpyfy
    states_np = np.concatenate(states_list, axis=0)
    acs_np = np.concatenate(acs_list, axis=0)
    all_rewards_np = np.concatenate(all_rewards_list, axis=0)
    latent_np = np.concatenate(latent_list, axis=0)
    ep_rewards_np = np.concatenate(ep_rewards_list, axis=0)
    ep_end_rewards_np = np.concatenate(ep_end_rewards_list, axis=0)
    # print(time_diffs_list[0].shape)
    time_diffs_np = np.concatenate(time_diffs_list, axis=1)
    # print(state_diffs_list[0].shape)
    state_diffs_np = np.concatenate(state_diffs_list, axis=0)
    # print(act_state_time_diffs_list[0].shape)
    act_state_time_diffs_np = np.concatenate(act_state_time_diffs_list, axis=1)
    # print(episode_sizes_list[0].shape)
    episode_sizes_np = np.concatenate(episode_sizes_list, axis=1)
    failed_ep_np = np.concatenate(failed_ep_list, axis=0)

    # _np = np.stack()

    assert np.sum(episode_sizes_np) == states_np.shape[0]

    # stats
    asd_dist = (np.mean(act_state_time_diffs_np), np.std(act_state_time_diffs_np))
    td_dist = (np.mean(time_diffs_np), np.std(time_diffs_np))
    sd_dist = (np.mean(state_diffs_np, axis=0), np.std(state_diffs_np, axis=0))
    s_dist = (np.mean(states_np, axis=0), np.std(states_np, axis=0))
    a_dist = (np.mean(acs_np, axis=0), np.std(acs_np, axis=0))
    e_dist = (np.mean(episode_sizes_np), np.std(episode_sizes_np))
    e_mm = (np.amin(episode_sizes_np), np.amax(episode_sizes_np))

    rew_dist = (np.mean(all_rewards_np, axis=0), np.std(all_rewards_np, axis=0))
    rew_mm = (np.amin(all_rewards_np, axis=0), np.amax(all_rewards_np, axis=0))

    ep_rew_dist = (np.mean(ep_rewards_np, axis=0), np.std(ep_rewards_np, axis=0))
    ep_rew_mm = (np.amin(ep_rewards_np, axis=0), np.amax(ep_rewards_np, axis=0))

    ep_end_rew_dist = (np.mean(ep_end_rewards_np, axis=0), np.std(ep_end_rewards_np, axis=0))
    ep_end_rew_mm = (np.amin(ep_end_rewards_np, axis=0), np.amax(ep_end_rewards_np, axis=0))

    # normalized
    states_normalized = np.divide(states_np - s_dist[0], s_dist[1])
    acs_normalized = np.divide(acs_np - a_dist[0], a_dist[1])

    print("#############################################################")
    print("## Saving %d states, %d actions, %d time diffs (skipped %d)" % (states_np.shape[0], acs_np.shape[0], time_diffs_np.size, skipped))
    print("## Total time elapsed: %.9f seconds" % np.sum(time_diffs_np))
    print("## Delta time (mu, sig): (%.9f, %.9f) seconds" % td_dist)
    print("## Act/state time offset (mu, sig): (%.9f, %.9d) seconds" % asd_dist)
    print("## State difference (mu, sig): (%s, %s) seconds" % sd_dist)
    print("## Episode sizes (mu, sig): (%s, %s)" % e_dist)
    print("## Episode sizes (min, max): (%s, %s)" % e_mm)
    print("#############################################################")
    print("## State distribution (mu, sig): (%s, %s)" % s_dist)
    print("## Act distribution (mu, sig): (%s, %s)" % a_dist)
    print("## All Reward distribution (mu, sig): (%s, %s)" % rew_dist)
    print("## All Reward distribution (min, max): (%s, %s)" % rew_mm)
    print("## Episode Reward distribution (mu, sig): (%s, %s)" % ep_rew_dist)
    print("## Episode Reward distribution (min, max): (%s, %s)" % ep_rew_mm)
    print("## End Reward distribution (mu, sig): (%s, %s)" % ep_end_rew_dist)
    print("## End Reward distribution (min, max): (%s, %s)" % ep_end_rew_mm)
    print("#############################################################")

    file_name = args.output_file
    if not train:
        file_name = add_val_to_filename(file_name)
    print("Saving to file: %s" % file_name)

    savemat(file_name, {
        'obs': states_np,
        'acs': acs_np,
        'rewards': all_rewards_np,
        'latent': latent_np,
        'ep_rewards': ep_rewards_np,
        'diffs': time_diffs_np,
        'state_diffs': state_diffs_np,
        'ac_state_time_diffs': act_state_time_diffs_np,
        'episode_sizes': episode_sizes_np,
        'state_dist': s_dist,
        'ac_dist': a_dist,
        'rew_dist': rew_dist,
        'rew_mm': rew_mm,
        'ep_rew_dist': ep_rew_dist,
        'ep_rew_mm': ep_rew_mm,
        'ep_end_rew_dist': ep_end_rew_dist,
        'ep_end_rew_mm': ep_end_rew_mm,
        'failed_ep': failed_ep_np,
    })


if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args(sys.argv[1:])

    print("+++++++++++++++++++ Training Set +++++++++++++++++++")
    bag_and_save(args, True)

    if args.do_val:
        print("+++++++++++++++++++ Validation Set +++++++++++++++++++")
        bag_and_save(args, False)