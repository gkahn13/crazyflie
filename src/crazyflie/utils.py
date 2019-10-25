import rospy
from std_msgs.msg import Bool
import time

START = True
STOP = not START

class SimpleTimer:
    def __init__(self, start=False):
        if start:
            self.start_time = time.time()
        self.all_times = []

    def reset(self):
        self.start_time = time.time()
        self.all_times = []

    def record(self):
        self.all_times.append(time.time())

    def format_string(self):
        assert hasattr(self, "start_time") and len(self.all_times) > 0 and self.start_time < self.all_times[0]

        string = ""
        all_with_start = [self.start_time] + self.all_times
        for i,t in enumerate(self.all_times):
            diff = t - all_with_start[i]
            string += "T%d: %.7f, " % (i, diff)

        string += "TOTAL: %.7f" % (self.all_times[-1] - self.start_time)

        return string

class BaseSigController:

    def __init__(self, ros_prefix):
        # true on the line means start, false means stop
        self.start_stop_msg = None

    def start_stop_cb(self, msg):
        self.start_stop_msg = msg

    # blocking
    def wait_sig(self, start_else_stop, rate):
        # wait for external_sig to start
        while self.start_stop_msg is None or self.start_stop_msg.data != start_else_stop:
            rate.sleep()

    def reset_sig(self):
        self.start_stop_msg = None

    def sender_stopped(self):
        return self.start_stop_msg is not None and self.start_stop_msg.data == STOP

class ServantSigController(BaseSigController):

    def __init__(self, ros_prefix):
        # true on the line means start, false means stop
        super().__init__(ros_prefix)
        self._ros_master_start_stop_sig = rospy.Subscriber(ros_prefix + "master/signal", Bool, self.start_stop_cb)
        self._ros_servant_start_stop_sig = rospy.Publisher(ros_prefix + "servant/signal", Bool, queue_size=10)

    def send_sig(self, start_else_stop):
        self._ros_servant_start_stop_sig.publish(start_else_stop)

class MasterSigController(BaseSigController):

    def __init__(self, ros_prefix):
        # true on the line means start, false means stop
        super().__init__(ros_prefix)
        self._ros_servant_start_stop_sig = rospy.Subscriber(ros_prefix + "servant/signal", Bool, self.start_stop_cb)
        self._ros_master_start_stop_sig = rospy.Publisher(ros_prefix + "master/signal", Bool, queue_size=10)

    def send_sig(self, start_else_stop):
        self._ros_master_start_stop_sig.publish(start_else_stop)

    # send once every resend * 1/rate, rate is a rospy.Rate
    def wait_send_sig(self, start_else_stop, rate, resend=1):
        # wait for external_sig to start
        i = 0
        while self.start_stop_msg is None or self.start_stop_msg.data != start_else_stop:
            if i % resend == 0:
                self.send_sig(start_else_stop)
            rate.sleep()
            i += 1

