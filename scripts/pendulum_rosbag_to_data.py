import math
import numpy as np
import rosbag
from scipy.io import savemat
import sys
import argparse


# def get_rosbag_file_name(bag_num, folder):
#     return folder + '/rosbag' + '0' * (4-len(bag_num)) + bag_num + '.bag'

def make_parser():
    # parse inputs
    parser = argparse.ArgumentParser(description='script to read and combine rosbag data',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-l','--input_list', nargs='+', help='list of input rosbags', required=True)
    parser.add_argument('-o', '--output_file', type=str, default='rosbag_data.mat',
                        help='output file for storing mat data')
    parser.add_argument('-n', '--normalize', action='store_true', help='normalize rosbag states to known pixel width, height, and max area')
    # parser.add_argument('-p', '--ros_prefix', type=str, default='',
    #                     help='prefix for crazyflie data in bag file')
    return parser

if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args(sys.argv[1:])

    topics = ['target_vector', 'motion']
    def get_intersecting_topics(bag_topics):
        final_topics = []
        for t in bag_topics:
            for subt in topics:
                if subt in t:
                    final_topics.append(t)
                    break
        return final_topics

    states, acs = [], []
    state = np.zeros(3)
    ac = np.zeros(3)
    time_diffs = []

    if args.normalize:
        print("Normalizing on")
        coef_cx = 1./640
        coef_cy = 1./480
        coef_area = 1./(640*480)
    else:
        coef_cx = coef_cy = coef_area = 1.
    
    skipped = 0
    # iterate through all the input bag files
    for filename in args.input_list:
        prev_time = None
        prev_save_time = None

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

        i = 0

        # rosbag exists, parse through topics
        for topic, msg, t in bag.read_messages(topics=relevant_topics):
            # if 'motion' in topic:
            #     print(topic, msg.stamp.seq, str(msg.stamp.stamp.secs) + "." + str(msg.stamp.stamp.nsecs))
            # else:
            #     print(topic, msg.header.seq, str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs))

            if 'target_vector' in topic:
                # only nonzero target vectors
                # if not (msg.vector.x == 0 and msg.vector.y == 0):
                # dup check
                if msg.header.seq in seq_nos[topic]:
                    continue
                else:
                    seq_nos[topic].add(msg.header.seq)

                seq_nos[topic].add(msg.header.seq)
                state[0] = msg.vector.x * coef_cx
                state[1] = msg.vector.y * coef_cy
                state[2] = msg.vector.z * coef_area
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

                    if state[0] == 0 and state[1] == 0:
                        skipped += 1
                    else:

                        # append action and latest state
                        states.append(np.copy(state))
                        acs.append(np.copy(ac))
                        if i > 0:
                            time_diffs.append((msg_time-prev_time))

                        i += 1

                    prev_time = msg_time


                # if prev_save_time:
                #     print('Time since last bag', (prev_time-prev_save_time)/1e9)
                # prev_save_time = prev_time
        bag.close()

    print("#############################################################")
    print("## Saving %d states, %d actions, %d time diffs (skipped %d)" % (len(states), len(acs), len(time_diffs), skipped))
    print("## Total time elapsed: %.9f seconds" % np.sum(time_diffs))
    print("#############################################################")

    savemat(args.output_file, {'obs': states, 'acs': acs, 'diffs': time_diffs})
