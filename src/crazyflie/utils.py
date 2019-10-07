import time

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