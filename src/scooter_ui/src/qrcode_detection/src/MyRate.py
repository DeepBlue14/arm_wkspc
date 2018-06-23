# [ml] mike lunderville
# MyRate class that works similiar to rospy.Rate, but not using ROS time

import time

class MyRate(object):

    def __init__(self, hz):
        self.sleep_dur = 1.0000000000 / hz
        self.last_time = time.time()

    def sleep(self):
        current_time = time.time()
        # Time may jump backwards (yay time travel)
        if self.last_time > current_time:
            self.last_time = current_time
            # Time spent calculating that should not be slept for
            elapsed = current_time - self.last_time
            # If elapsed is more than the sleep was supposed to be, don't sleep
            if elapsed < self.sleep_dur:
                time.sleep(self.sleep_dur - elapsed)
                # After everything is done, set the last_time to be the time
                # that was clocked at the beginning of this funciton call.
                self.last_time += self.sleep_dur
                # If the loop calling this is too slow or something like that, fix the time
                if current_time - self.last_time > 2 * self.sleep_dur:
                    self.last_time = current_time

