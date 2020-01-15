#!/usr/bin/env python2

import rospy
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import *

class AttitudeController:
	def __init__(self):
		rospy.init_node('attitude_controller', anonymous=True)
		self.pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		self.yaw = 1.57
		self.previous_time = rospy.Time.now()
		rospy.Subscriber('mavros/rc/in', RCIn, self.callback)
		rospy.spin()

	def callback(self, rc):
		#rospy.loginfo('{} {} {} {}'.format(rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3]))
		cmd = AttitudeTarget()
		Ts = (rospy.Time.now() - self.previous_time).to_sec()
		self.previous_time = rospy.Time.now()
		cmd.header.stamp = rospy.Time.now()
		cmd.type_mask = AttitudeTarget.IGNORE_ROLL_RATE+AttitudeTarget.IGNORE_PITCH_RATE
		#ROS use ENU convention
		roll = .6*(rc.channels[1]-1492.)/500.
		pitch = -.6*(rc.channels[2]-1493.)/500.
		yawrate = -3.*(rc.channels[3]-1498.)/500.
		self.yaw = self.yaw + yawrate*Ts
		thrust = (rc.channels[0]-1000.)/1200.
		rospy.loginfo('Ts = {Ts:.3f} r = {r:.2f} p = {p:.2f} y = {y:.2f} t = {t:.2f}'.format(Ts=Ts, r=roll, p=pitch, y=self.yaw, t=thrust))
		q = quaternion_from_euler(roll, pitch, self.yaw)
		cmd.orientation.x = q[0]
		cmd.orientation.y = q[1]
		cmd.orientation.z = q[2]
		cmd.orientation.w = q[3]
		cmd.body_rate.x=0.
		cmd.body_rate.y=0.
		cmd.body_rate.z=yawrate
		cmd.thrust = thrust
		self.pub.publish(cmd)

if __name__ == '__main__':
	try:
		controller = AttitudeController()
	except rospy.ROSInterruptException:
		pass
