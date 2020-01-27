import math
import numpy as np
import rospy
import threading
import time
from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan

def stop_car():
    print "Stopping car."
    pub = rospy.Publisher('drive_parameters', drive_params, queue_size=5)
    for i in range(50):
        msg = drive_params()
        msg.angle = 0.5
        msg.velocity = 0.0
        pub.publish(msg)

class SpeedTesterDriving(object):
    """ Holds configuration options and some state for controlling the car
    using the simplified obstacle-bloating algorithm I'm calling "disparity
    extending" for now, since I don't know if there's an already established
    term for it. """

    def __init__(self):
        """ Initializes the class with default values, for our 1/10 scale
        traxxas car controlled using a FOCBox. """
        self.velocity = 0.5
        # We'll use this lock to essentially drop LIDAR packets if we're still
        # processing an older one.
        self.lock = threading.Lock()
        # The number of seconds to allow the car to accelerate for before
        # measuring speed.
        self.accelerate_duration = 1.0
        self.measurement_duration = 1.0
        self.start_time = None
        self.accelerate_end_time = None
        self.start_distance = None
        self.should_stop = False
        self.pub_drive_param = rospy.Publisher('drive_parameters',
            drive_params, queue_size=5)

    def run(self):
        """ Starts running the car until we want it to stop! """
        start_time = time.time()
        rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        rospy.spin()

        # rospy.spin() will run until Ctrl+C or something similar causes it to
        # return. So, at this point we know the script should exit.
        self.should_stop = True
        # Try, with a timeout, to acquire the lock and block the other threads.
        got_lock = False
        for i in range(10):
            got_lock = self.lock.acquire(False)
            if got_lock:
                break
            time.sleep(0.05)
        duration = time.time() - start_time
        self.pub_drive_param.publish(0, 0)
        self.pub_drive_param.unregister()
        drop_rate = float(self.dropped_packets) / float(self.total_packets)
        print "Done processing. Ran for %fs" % (duration,)
        if got_lock:
            self.lock.release()

    def lidar_callback(self, lidar_data):
        """ This is asynchronously called every time we receive new LIDAR data.
        """
        # If the lock is currently locked, then previous LIDAR data is still
        # being processed.
        if not self.lock.acquire(False):
            return
        if self.should_stop:
            return
        target_velocity = self.velocity
        distances = lidar_data.ranges
        forward_distance = distances[len(distances) / 2]
        current_time = time.time()
        # If the car just started moving, record the overall start time
        if self.start_time is None:
            self.start_time = current_time
        # If the acceleration duration has elapsed, then record the time and
        # distance at which we'll start the speed calculation.
        if (self.start_distance is None) and current_time >= (self.start_time +
            self.accelerate_duration):
            self.accelerate_end_time = current_time
            self.start_distance = forward_distance
        # If the measurement duration has elapsed, then calculate the speed
        # based on the distance that we moved.
        if (self.accelerate_end_time is not None) and (current_time >=
            (self.accelerate_end_time + self.measurement_duration)):
            self.should_stop = True
            target_velocity = 0.0
            time_elapsed = current_time - self.accelerate_end_time
            distance_elapsed = self.start_distance - forward_distance
            speed = distance_elapsed / time_elapsed
            print "Current distance: %f. Traveled %fm in %fs. Speed: %f m/s" % (
                forward_distance, distance_elapsed, time_elapsed, speed)
        msg = drive_params()
        msg.angle = 0.5
        msg.velocity = target_velocity
        if forward_distance < 3.0:
            msg.velocity = 0.0
        self.pub_drive_param.publish(msg)
        self.lock.release()


def run_main():
    rospy.init_node('measure_speed', anonymous=True)
    controller = SpeedTesterDriving()
    controller.run()

if __name__ == '__main__':
    run_main()
