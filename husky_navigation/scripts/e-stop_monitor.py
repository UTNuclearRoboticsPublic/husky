#!/usr/bin/env python
__author__ = "Ben Ebersole"
__copyright__ = "Copyright 2015, The University of Texas at Austin, " \
                "Nuclear Robotics Group"
__credits__ = "Ben Ebersole"
__license__ = "BSD"
__version__ = "0.0.2"
__maintainer__ = "Ben Ebersole"
__email__ = "ebersole@utexas.edu"
__status__ = "Production"
__doc__ = """This node filters the IMU and Odometry data so that the EKF pose
	     estimate ignores IMU drift while the Odometry data shows no motion.
             It also republishes the encoder data in joint_states with the
             drivetrain correction factor and allows a service to reset the values"""

import rospy
from actionlib_msgs.msg import GoalID
from husky_msgs.msg import HuskyStatus

class Monitor():
    def __init__(self):
        rospy.Subscriber("status",HuskyStatus,self.status_callback)
        self.cancel_pub = rospy.Publisher("move_base/cancel",GoalID,queue_size=10)

    def status_callback(self,data):
        if data.e_stop:
            msg = GoalID()
            self.cancel_pub.publish(msg)

if __name__ == '__main__':
	try:
		rospy.init_node("estop_monitor")
		test = Monitor()

		rospy.spin()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise
