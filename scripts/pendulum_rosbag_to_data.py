import math
import numpy as np
import rosbag
from scipy.io import savemat
import sys
import argparse
from scipy.signal import correlate

# def get_rosbag_file_name(bag_num, folder):
#     return folder + '/rosbag' + '0' * (4-len(bag_num)) + bag_num + '.bag'

def make_parser():
    # parse inputs
    parser = argparse.ArgumentParser(description='script to read and combine rosbag data',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-l','--input_list', nargs='+', help='list of input rosbags', required=True)
    parser.add_argument('-lv','--latent_var', type=str, help='latent var for this episode', default='None')
    parser.add_argument('-o', '--output_file', type=str, default='rosbag_data.mat',
                        help='output file for storing mat data')
    parser.add_argument('-n', '--normalize', action='store_true', help='normalize rosbag states to known pixel width, height, and max area')
    parser.add_argument('-nc', type=int, help='how many correlation components to show', default=2)
    parser.add_argument('-c', '--correlate', action='store_true', help='plot correlation of rosbag states')
    parser.add_argument('-rew', '--rewards', action='store_true', help='sum episode rewards in bag if present')
    parser.add_argument('-v', '--validation_frac', type=float, help='validation size fraction', default=0.)
    parser.add_argument('-al', '--artificial_lag', type=int, help='how many time steps to delay actions', default=0)
    # parser.add_argument('-p', '--ros_prefix', type=str, default='',
    #                     help='prefix for crazyflie data in bag file')
    return parser

def bag_and_save(args, train):

    topics = ['target_vector', 'motion']
    if args.rewards:
        topics += ['reward_vector']

    if args.latent_var == 'None':
        print('WARNING: Attempting to read lv from bag')
        topics += ['latent_vector']
        raise NotImplementedError
    else:
        const_latent_var = float(args.latent_var) # will error if not a float

    def get_intersecting_topics(bag_topics):
        final_topics = []
        for t in bag_topics:
            for subt in topics:
                if subt in t:
                    final_topics.append(t)
                    break
        return final_topics

    states, acs = [], []
    latent = []
    state = np.zeros(3)
    ac = np.zeros(3)
    last_n_acs = []
    last_n_ac_times = []
    time_diffs = []
    state_diffs = []
    act_state_time_diffs = []
    state_counts = []
    episode_sizes = []
    all_rewards = []
    ep_rewards = []
    ep_end_rewards = []
    failed_ep = []

    if args.normalize:
        print("Normalizing on")
        coef_cx = 1./640
        coef_cy = 1./480
        coef_area = 1./(640*480)
    else:
        coef_cx = coef_cy = coef_area = 1.
    
    skipped = 0

    num_bags = math.ceil(len(args.input_list) * (1-args.validation_frac))
    if train:
        bags = args.input_list[:num_bags] # (1-val frac)
    else:
        bags = args.input_list[num_bags:]

    # iterate through all the input bag files
    for filename in bags:
        prev_time = None
        state_time = None
        prev_state_time = None
        prev_state = None
        prev_save_time = None
        cur_rew = np.zeros(3)

        print("Reading:", filename)

        # try loading rosbag
        try:
            bag = rosbag.Bag(filename)
        except:
            print('No bagfile with name', filename)
            continue

        all_ros_topics = bag.get_type_and_topic_info()[1].keys()
        relevant_topics = get_intersecting_topics(all_ros_topics)
        seq_nos = {topic: set() for topic in relevant_topics}
        print("TOPICS:", relevant_topics)

        assert len(relevant_topics) == len(topics)

        i = 0
        tmp_state_count = 0

        sum_rewards = np.zeros(3)

        tuples = {}
        for rt in relevant_topics:
            tuples[rt] = []

        # topic, msg, t
        ordered = sorted(bag.read_messages(topics=relevant_topics), key=lambda tup: tup[2])


        # rosbag exists, parse through topics
        for topic, msg, t in ordered:
            # if 'motion' in topic:
            #     print(topic, msg.stamp.seq, str(msg.stamp.stamp.secs) + "." + str(msg.stamp.stamp.nsecs))
            # else:
            #     print(topic, msg.header.seq, str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs))

            if 'target_vector' in topic:
                # only nonzero target vectors
                # if not (msg.vector.x == 0 and msg.vector.y == 0):
                # dup check

                state_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
                if prev_state_time:
                    if state_time < prev_state_time:
                        continue

                prev_state_time = state_time
                state[0] = msg.vector.x * coef_cx
                state[1] = msg.vector.y * coef_cy
                state[2] = msg.vector.z * coef_area
                tmp_state_count += 1

            elif 'motion' in topic:
                msg_time = msg.stamp.stamp.secs + msg.stamp.stamp.nsecs/1e9
                if msg.is_flow_motion:
                    # dup check
                    if msg_time in seq_nos[topic]:
                        continue
                    else:
                        seq_nos[topic].add(msg_time)

                    # populate in action
                    ac[0] = msg.x
                    ac[1] = msg.y
                    ac[2] = msg.dz

                    last_n_acs.append(ac.copy())
                    last_n_ac_times.append(msg_time)
                    if len(last_n_acs) > args.artificial_lag + 1:
                        last_n_acs.pop(0)
                        last_n_ac_times.pop(0)

                    # skip if we don't have enough artificial lag terms
                    if (state[0] == 0 and state[1] == 0) or len(last_n_acs) < args.artificial_lag + 1:
                        skipped += 1
                    else:
                        r_ac = last_n_acs[-1-args.artificial_lag]
                        r_ac_time = last_n_ac_times[-1-args.artificial_lag]

                        # append action and latest state
                        states.append(np.copy(state))
                        acs.append(np.copy(r_ac))
                        latent.append(np.array([const_latent_var]))

                        state_counts.append(tmp_state_count)

                        if prev_time is not None:
                            time_diffs.append((msg_time-prev_time))
                        if state_time is not None:
                            act_state_time_diffs.append(abs(r_ac_time-state_time))
                        if prev_state is not None:
                            state_diffs.append(state - prev_state)
                        i += 1
                        prev_state = states[-1]

                        all_rewards.append(np.copy(cur_rew))

                    prev_time = msg_time
                    tmp_state_count = 0

            elif 'reward_vector' in topic:
                vec = msg.vector
                cur_rew = np.array([vec.x, vec.y, vec.z])
                sum_rewards += cur_rew # episode rewards

        episode_sizes.append(i)
        ep_rewards.append(sum_rewards)
        ep_end_rewards.append(all_rewards[-1])
        if state[0] < 1e-6 and state[1] < 1e-6: # (failed episode)
            failed_ep.append(np.ones((1,)))
        else:
            failed_ep.append(np.zeros((1,)))

                # if prev_save_time:
                #     print('Time since last bag', (prev_time-prev_save_time)/1e9)
                # prev_save_time = prev_time
        bag.close()

    # numpyfy
    act_state_time_diffs_np = np.stack(act_state_time_diffs)
    time_diffs_np = np.stack(time_diffs)
    state_diffs_np = np.stack(state_diffs)
    state_counts_np = np.stack(state_counts)
    states_np = np.stack(states)
    acs_np = np.stack(acs)
    latent_np = np.stack(latent)
    episode_sizes_np = np.stack(episode_sizes)
    all_rewards_np = np.stack(all_rewards)
    ep_rewards_np = np.stack(ep_rewards)
    ep_end_rewards_np = np.stack(ep_end_rewards)
    failed_ep_np = np.stack(failed_ep)
    # _np = np.stack()

    assert np.sum(episode_sizes_np) == states_np.shape[0]

    # stats
    asd_dist = (np.mean(act_state_time_diffs_np), np.std(act_state_time_diffs_np))
    td_dist = (np.mean(time_diffs_np), np.std(time_diffs_np))
    sd_dist = (np.mean(state_diffs_np, axis=0), np.std(state_diffs_np, axis=0))
    sc_dist = (np.mean(state_counts_np, axis=0), np.std(state_counts_np, axis=0))
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
    states_normalized = np.divide(states - s_dist[0], s_dist[1])
    acs_normalized = np.divide(acs - a_dist[0], a_dist[1])

    print("#############################################################")
    print("## Saving %d states, %d actions, %d time diffs (skipped %d)" % (len(states), len(acs), len(time_diffs), skipped))
    print("## Total time elapsed: %.9f seconds" % np.sum(time_diffs))
    print("## Delta time (mu, sig): (%.9f, %.9f) seconds" % td_dist)
    print("## Act/state time offset (mu, sig): (%.9f, %.9d) seconds" % asd_dist)
    print("## State difference (mu, sig): (%s, %s) seconds" % sd_dist)
    print("## State counts (mu, sig): (%s, %s)" % sc_dist)
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
        if file_name[-4:] == ".mat":
            file_name = file_name[:-4]
        file_name += "_val.mat"
    print("Saving to file: %s" % file_name)

    savemat(file_name, {
        'obs': states_np,
        'acs': acs_np,
        'rewards': all_rewards_np,
        'latent': latent_np,
        'ep_rewards': ep_rewards_np,
        'ep_end_rewards': ep_end_rewards_np,
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


    # correlation
    if args.correlate:
        from sklearn.cross_decomposition import CCA
        import matplotlib.pyplot as plt

        cca = CCA(n_components=args.nc)
        cstates, cacs = cca.fit_transform(states_normalized, acs_normalized)
        # plt.plot(corr)
        gspec = [args.nc,args.nc]

        f = plt.figure(constrained_layout=True, figsize=(16, 10))
        f.suptitle('Correlation')
        gs = f.add_gridspec(*gspec)

        for d in range(args.nc):
            s = cstates[:, d]
            for u in range(args.nc):
                a = cacs[:, u]

                # f = plt.figure(constrained_layout=True, figsize=(16, 8))
                # f.suptitle('Uncertainties (%d STDEV, Dim: %d, U: %s)' % (args.numdev, d, u_name))
                # gs = f.add_gridspec(*gspec)
                # ax = f.add_subplot(gs[0, 0])
                ax = f.add_subplot(gs[d, u])
                
                ax.set_title("Dim %d: AcDim: %d" % (d, u))

                ax.scatter(s, a, color='blue')
                ax.set_xlabel('states[%d]' % d)
                ax.set_ylabel('acs[%d]' % u)

        print("Plotting...")
        plt.show()
        print("Done.")



if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args(sys.argv[1:])

    print("+++++++++++++++++++ Training Set +++++++++++++++++++")
    bag_and_save(args, True)

    if args.validation_frac > 0:
        print("+++++++++++++++++++ Validation Set +++++++++++++++++++")
        bag_and_save(args, False)