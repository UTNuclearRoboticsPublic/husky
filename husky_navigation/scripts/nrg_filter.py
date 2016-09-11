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
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
import tf
from math import cos, sin, pi

class Filter():
	def __init__(self):
		self.stationary = True
		self.filter = False
                self.reset_odom = False
		self.reset_joints = False

		self.joints_offset = (0,0,0,0)
		self.odom_trans_offset = (0,0,0)
                self.odom_rot_offset = 0

		rospy.Subscriber("joint_states", JointState, self.joint_callback)
		rospy.Subscriber("husky_velocity_controller/odom", Odometry, self.odom_callback)
		rospy.Subscriber("imu/data", Imu, self.imu_callback)

		rospy.Service('~enable', Empty, self.HandleEnable)
		rospy.Service('~disable', Empty, self.HandleDisable)
                rospy.Service('~reset', Empty, self.HandleReset)

		self.odom_pub = rospy.Publisher("~odom",Odometry,queue_size = 10)
                self.hec_odom_pub = rospy.Publisher("~hector_odom",Odometry,queue_size = 10)
		self.imu_pub = rospy.Publisher("~imu",Imu,queue_size = 10)
                self.joints_pub = rospy.Publisher("~joints",JointState,queue_size = 10)
               
                self.encoder_modifier = 0.527

	def odom_callback(self,data):
		odom = data
		if (self.stationary == True and self.filter == True):
			covar = list(odom.pose.covariance)
			covar[35] = 0.001
			odom.pose.covariance = tuple(covar)
			covar = list(odom.twist.covariance)
			covar[35] = 0.001
			odom.twist.covariance = tuple(covar)

                quat_live = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
		
		if self.reset_odom:
			euler = tf.transformations.euler_from_quaternion(quat_live)
			self.odom_trans_offset = (odom.pose.pose.position.x, odom.pose.pose.position.y, 0)
			self.odom_rot_offset = euler[2]
                        self.reset_odom = False
		
		quat_offset = tf.transformations.quaternion_from_euler(0,0,self.odom_rot_offset)
                quat_new = tf.transformations.quaternion_multiply(quat_live,tf.transformations.quaternion_inverse(quat_offset))

		tempx = odom.pose.pose.position.x - self.odom_trans_offset[0]
		tempy = odom.pose.pose.position.y - self.odom_trans_offset[1]

		odom.pose.pose.position.x = tempx*cos(self.odom_rot_offset)+tempy*sin(self.odom_rot_offset)
		odom.pose.pose.position.y = -tempx*sin(self.odom_rot_offset)+tempy*cos(self.odom_rot_offset)
		odom.pose.pose.orientation.x = quat_new[0]
		odom.pose.pose.orientation.y = quat_new[1]
		odom.pose.pose.orientation.z = quat_new[2]
		odom.pose.pose.orientation.w = quat_new[3]

#                hec_odom = odom
#                hec_odom.header.frame_id = 'map'
#		print odom
		self.odom_pub.publish(odom)
#                self.hec_odom_pub.publish(hec_odom)

	def imu_callback(self,data):
		imu = data
		if (self.stationary == True and self.filter == True):
			covar = list(imu.orientation_covariance)
			covar[0] = 1.0
			covar[4] = 1.0
			covar[8] = 1.0
			imu.orientation_covariance = tuple(covar)
			covar = list(imu.angular_velocity_covariance)
			covar[0] = 1.0
			covar[4] = 1.0
			covar[8] = 1.0
			imu.angular_velocity_covariance = tuple(covar)
			covar = list(imu.linear_acceleration_covariance)
			covar[0] = 1.0
			covar[4] = 1.0
			covar[8] = 1.0
			imu.linear_acceleration_covariance = tuple(covar)
		self.imu_pub.publish(imu)

	def joint_callback(self,data):
		joints = data
                pos = list(joints.position)
                vels = list(joints.velocity)
 		for ii in range(4):
			pos[ii] = pos[ii] * self.encoder_modifier
			vels[ii] = vels[ii] * self.encoder_modifier

                if self.reset_joints:
			self.joints_offset = tuple(pos)
                        self.reset_joints = False

                joints_offset = list(self.joints_offset)

		for ii in range(4):                        
			pos[ii] = pos[ii] - joints_offset[ii]
                joints.position = tuple(pos) 
		joints.velocity = tuple(vels)

		if joints.name[0] == 'front_left_wheel':
			if (joints.velocity[0] == 0.0 and joints.velocity[1] == 0.0 and
				joints.velocity[2] == 0.0 and joints.velocity[3] == 0.0):
				self.stationary = True
			else:
				self.stationary = False
		self.joints_pub.publish(joints)

	def HandleEnable(self,req):
		if self.filter == True:
			rospy.loginfo("Filter already enabled!")
			return EmptyResponse()
		else:
			self.filter = True
			rospy.loginfo("Enabling nrg_nav filter")
			return EmptyResponse()

	def HandleDisable(self,req):
		if self.filter == False:
			rospy.loginfo("Filter already disabled!")
			return EmptyResponse()
		else:
			self.filter = False
			rospy.loginfo("Disabling nrg_nav filter")
			return EmptyResponse()

	def HandleReset(self,req):
		self.reset_joints = True
		self.reset_odom = True
		rospy.loginfo("Resetting nrg-filtered joints & odometry")
		return EmptyResponse()

if __name__ == '__main__':
	try:
		rospy.init_node("nrg_nav_filter")
		test = Filter()

		rospy.spin()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise
