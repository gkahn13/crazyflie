import math
import numpy as np
import rosbag
from scipy.io import savemat
import sys
import argparse

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = (math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = (math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = (math.atan2(t3, t4))

    return X, Y, Z

# def get_rosbag_file_name(bag_num, folder):
#     return folder + '/rosbag' + '0' * (4-len(bag_num)) + bag_num + '.bag'

def make_parser():
    # parse inputs
    parser = argparse.ArgumentParser(description='script to read and combine rosbag data',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-l','--input_list', nargs='+', help='list of input rosbags', required=True)
    parser.add_argument('-o', '--output_file', type=str, default='rosbag_data.mat',
                        help='output file for storing mat data')
    # parser.add_argument('-p', '--ros_prefix', type=str, default='',
    #                     help='prefix for crazyflie data in bag file')
    return parser

if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args(sys.argv[1:])

    topics = ['pose', 'twist', 'motion']
    def get_intersecting_topics(bag_topics):
        final_topics = []
        for t in bag_topics:
            for subt in topics:
                if subt in t:
                    final_topics.append(t)
                    break
        return final_topics

    states, acs = [], []
    state = np.zeros(12)
    ac = np.zeros(3)
    coeff = np.pi / 180.0
    time_diffs = []
    
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
            if 'motion' in topic:
                print(topic, msg.stamp.seq, msg.stamp.stamp.secs)
            else:
                print(topic, msg.header.seq, msg.header.stamp.secs)

            i += 1

            if i > 100:
                import ipdb; ipdb.set_trace()

            if 'motion' in topic:
                if msg.stamp.stamp.nsecs in seq_nos[topic]:
                    continue
                else:
                    seq_nos[topic].add(msg.stamp.stamp.nsecs)
            elif 'motion' not in topic:
                if msg.header.seq in seq_nos[topic]:
                    continue
                else:
                    curr_time = msg.header.stamp.nsecs
                    if prev_time:
                        time_diffs.append((curr_time-prev_time)/1e9)
                        #print('Time difference', (curr_time-prev_time)/1e9)
                    prev_time = curr_time
                    seq_nos[topic].add(msg.header.seq)
            if 'pose' in topic:
                #xyz
                state[0] = msg.pose.position.x
                state[1] = msg.pose.position.y
                state[2] = msg.pose.position.z
                #rpy
                state[6], state[7], state[8] = quaternion_to_euler( msg.pose.orientation.x,
                                                                    msg.pose.orientation.y,
                                                                    msg.pose.orientation.z,
                                                                    msg.pose.orientation.w)
            elif 'twist' in topic:
                #xyz dot
                state[3] = msg.twist.linear.x
                state[4] = msg.twist.linear.y
                state[5] = msg.twist.linear.z
                #rpy dot
                state[9] = coeff * msg.twist.angular.x
                state[10] = coeff * msg.twist.angular.y
                state[11] = coeff * msg.twist.angular.z
            elif 'motion' in topic:
                ac[0] = coeff * msg.x
                ac[1] = coeff * msg.y
                ac[2] = coeff * msg.yaw
                states.append(np.copy(state))
                acs.append(np.copy(ac))
                # if prev_save_time:
                #     print('Time since last bag', (prev_time-prev_save_time)/1e9)
                # prev_save_time = prev_time
        bag.close()
    # savemat(folder+'_rosbag_times.mat', {'diffs': time_diffs})
    # savemat(folder+'_rosbag_data.mat', {'obs': states, 'acs': acs})
