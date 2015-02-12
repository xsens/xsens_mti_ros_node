#!/usr/bin/env python
import roslib; roslib.load_manifest('xsens_driver')
import rospy
import select
import mtdevice
import math
import pdb

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped
from gps_common.msg import GPSFix, GPSStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from custom_msgs.msg import sensorSample

# transform Euler angles or matrix into quaternions
from math import pi, radians
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix

import numpy

def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v

class XSensDriver(object):

	ENU = numpy.identity(3)
	NED = numpy.array([[0, 1, 0], [ 1, 0, 0], [0, 0, -1]])
	NWU = numpy.array([[0, 1, 0], [-1, 0, 0], [0, 0,  1]])

	def __init__(self):

		device = get_param('~device', 'auto')
		baudrate = get_param('~baudrate', 0)
		if device=='auto':
			devs = mtdevice.find_devices()
			if devs:
				device, baudrate = devs[0]
				rospy.loginfo("Detected MT device on port %s @ %d bps"%(device,
						baudrate))
			else:
				rospy.logerr("Fatal: could not find proper MT device.")
				rospy.signal_shutdown("Could not find proper MT device.")
				return
		if not baudrate:
			baudrate = mtdevice.find_baudrate(device)
		if not baudrate:
			rospy.logerr("Fatal: could not find proper baudrate.")
			rospy.signal_shutdown("Could not find proper baudrate.")
			return

		rospy.loginfo("MT node interface: %s at %d bd."%(device, baudrate))
		self.mt = mtdevice.MTDevice(device, baudrate)

		self.frame_id = get_param('~frame_id', '/mti/data')

		frame_local     = get_param('~frame_local'    , 'ENU')
		frame_local_imu = get_param('~frame_local_imu', 'ENU')

		if   frame_local == 'ENU':
			R = XSensDriver.ENU
		elif frame_local == 'NED':
			R = XSensDriver.NED
		elif frame_local == 'NWU':
			R = XSensDriver.NWU

		if   frame_local_imu == 'ENU':
			R_IMU = XSensDriver.ENU
		elif frame_local_imu == 'NED':
			R_IMU = XSensDriver.NED
		elif frame_local_imu == 'NWU':
			R_IMU = XSensDriver.NWU

		self.R = R.dot(R_IMU.transpose())

		self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
		self.diag_msg = DiagnosticArray()
		self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
				message='No status information')
		self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
				message='No status information')
		self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
				message='No status information')
		self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

		self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10) #IMU message
		self.ss_pub = rospy.Publisher('xsens/sensorSample', sensorSample, queue_size=10) # sensorSample
		self.mag_pub = rospy.Publisher('magnetic', Vector3Stamped, queue_size=10) # magnetic
		self.baro_pub = rospy.Publisher('pressure', Float64, queue_size=10) # baro
		self.gnssPvt_pub = rospy.Publisher('fix', GPSFix, queue_size=10) # GNSS PVT
		self.gnssSatinfo_pub = rospy.Publisher('fix_extended', GPSStatus, queue_size=10) # GNSS SATINFO
		self.ori_pub = rospy.Publisher('orientation', Vector3Stamped, queue_size=10) # XKF/XEE orientation
		self.vel_pub = rospy.Publisher('velocity', Vector3Stamped, queue_size=10) # XKF/XEE velocity
		self.pos_pub = rospy.Publisher('position', Vector3Stamped, queue_size=10) # XKF/XEE position
		
		self.temp_pub = rospy.Publisher('temperature', Float32, queue_size=10)	# decide type
		



	def spin(self):
		try:
			while not rospy.is_shutdown():
				self.spin_once()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):
		
		# get data
		data = self.mt.read_measurement()
		# common header
		h = Header()
		h.stamp = rospy.Time.now()
		h.frame_id = self.frame_id

		# get data (None if not present)
		temp = data.get('Temp')	# float
		orient_data = data.get('Orientation Data')
		velocity_data = data.get('Velocity')
		position_data = data.get('Latlon')
		altitude_data = data.get('Altitude')
		acc_data = data.get('Acceleration')
		gyr_data = data.get('Angular Velocity')
		mag_data = data.get('Magnetic')
		pressure_data = data.get('Pressure')
		time_data = data.get('Timestamp')
		#rawgnss_data = data.get('RAWGPS')
		status = data.get('Status')	# int

		# create messages and default values
		"Imu message supported with Modes 1 & 2"
		imu_msg = Imu()
		pub_imu = False
		"SensorSample message supported with Mode 2"
		ss_msg = sensorSample()
		pub_ss = False
		"Mag message supported with Modes 1 & 2"
		mag_msg = Vector3Stamped()
		pub_mag = False
		"Baro in meters"
		baro_msg = Float64()
		pub_baro = False
		"GNSS message supported only with MTi-G-7xx devices"
		"Valid only for modes 1 and 2"
		gnssPvt_msg = GPSFix()
		pub_gpsPvt = False
		gnssSatinfo_msg = GPSStatus()
		pub_gnssSatinfo = False		
		# All filter related outputs
		"Supported in mode 3"
		ori_msg = Vector3Stamped()
		pub_ori = False
		"Supported in mode 3 for MTi-G-7xx devices"
		vel_msg = Vector3Stamped()
		pub_vel = False
		"Supported in mode 3 for MTi-G-7xx devices"
		pos_msg = Vector3Stamped()
		pub_pos = False
		
		secs = 0
		nsecs = 0
		
		if time_data:
			# first getting the sampleTimeFine
			time = time_data['SampleTimeFine']
			secs = 100e-6*time
			nsecs = 1e5*time - 1e9*math.floor(secs)							
		
		if acc_data:
			if 'Delta v.x' in acc_data: # found delta-v's
				pub_ss = True
				ss_msg.internal.imu.dv.x = acc_data['Delta v.x']
				ss_msg.internal.imu.dv.y = acc_data['Delta v.y']
				ss_msg.internal.imu.dv.z = acc_data['Delta v.z']
				#all time assignments
				ss_msg.time.data.secs = secs
				ss_msg.time.data.nsecs = nsecs									
			elif 'accX' in acc_data: # found acceleration
				pub_imu = True
				imu_msg.linear_acceleration.x = acc_data['accX']
				imu_msg.linear_acceleration.y = acc_data['accY']
				imu_msg.linear_acceleration.z = acc_data['accZ']
				#all time assignments
				imu_msg.header.stamp.secs = secs
				imu_msg.header.stamp.nsecs = nsecs				
			else:
				raise MTException("Unsupported message in XDI_AccelerationGroup.")	
					
		if gyr_data:
			if 'Delta q0' in gyr_data: # found delta-q's
				pub_ss = True
				ss_msg.internal.imu.dq.w = gyr_data['Delta q0']
				ss_msg.internal.imu.dq.x = gyr_data['Delta q1']
				ss_msg.internal.imu.dq.y = gyr_data['Delta q2']
				ss_msg.internal.imu.dq.z = gyr_data['Delta q3']
			elif 'gyrX' in gyr_data: # found rate of turn
				pub_imu = True
				imu_msg.angular_velocity.x = gyr_data['gyrX']
				imu_msg.angular_velocity.y = gyr_data['gyrY']
				imu_msg.angular_velocity.z = gyr_data['gyrZ']
			else:
				raise MTException("Unsupported message in XDI_AngularVelocityGroup.")
		
		if mag_data:
			# magfield
			ss_msg.internal.mag.x = mag_msg.vector.x = mag_data['magX']
			ss_msg.internal.mag.y = mag_msg.vector.y = mag_data['magY']
			ss_msg.internal.mag.z = mag_msg.vector.z = mag_data['magZ']
			pub_mag = True						
			#all time assignments	
			mag_msg.header.stamp.secs = secs
			mag_msg.header.stamp.nsecs = nsecs		

			# to fix
		#if rawgps_data:
		#	if rawgps_data['bGPS']<self.old_bGPS:
		#		pub_gps = True
		#		# LLA
		#		xgps_msg.latitude = gps_msg.latitude = rawgps_data['LAT']*1e-7
		#		xgps_msg.longitude = gps_msg.longitude = rawgps_data['LON']*1e-7
		#		xgps_msg.altitude = gps_msg.altitude = rawgps_data['ALT']*1e-3
				# NED vel # TODO?
				# Accuracy
				# 2 is there to go from std_dev to 95% interval
		#		xgps_msg.err_horz = 2*rawgps_data['Hacc']*1e-3
		#		xgps_msg.err_vert = 2*rawgps_data['Vacc']*1e-3
		#	self.old_bGPS = rawgps_data['bGPS']
		if temp is not None:
			pub_temp = True
			temp_msg.data = temp
				
		
		if orient_data:
			if 'Q0' in orient_data:
				pub_imu = True
				imu_msg.orientation.x = orient_data['Q0']
				imu_msg.orientation.y = orient_data['Q1']
				imu_msg.orientation.z = orient_data['Q2']
				imu_msg.orientation.w = orient_data['Q3']
			elif 'Roll' in orient_data:
				pub_ori = True
				ori_msg.vector.x = orient_data['Roll']
				ori_msg.vector.y = orient_data['Pitch']
				ori_msg.vector.z = orient_data['Yaw']
				#all time assignments
				ori_msg.header.stamp.secs = secs
				ori_msg.header.stamp.nsecs = nsecs
			else:
				raise MTException('Unsupported message in XDI_OrientationGroup')

		if velocity_data:
			pub_vel = True
			vel_msg.vector.x = velocity_data['velX']
			vel_msg.vector.y = velocity_data['velY']
			vel_msg.vector.z = velocity_data['velZ']
			#all time assignments
			vel_msg.header.stamp.secs = secs
			vel_msg.header.stamp.nsecs = nsecs
							
		if position_data:
			pub_pos = True
			pos_msg.vector.x = position_data['lat']
			pos_msg.vector.y = position_data['lon']
			#all time assignments
			pos_msg.header.stamp.secs = secs
			pos_msg.header.stamp.nsecs = nsecs
		if altitude_data:
			pub_pos = True	
			pos_msg.vector.z = altitude_data['ellipsoid']
			
		#if status is not None:
		#	if status & 0b0001:
		#		self.stest_stat.level = DiagnosticStatus.OK
		#		self.stest_stat.message = "Ok"
		#	else:
		#		self.stest_stat.level = DiagnosticStatus.ERROR
		# 		self.stest_stat.message = "Failed"
		#	if status & 0b0010:
		#		self.xkf_stat.level = DiagnosticStatus.OK
		#		self.xkf_stat.message = "Valid"
		#	else:
		#		self.xkf_stat.level = DiagnosticStatus.WARN
		#		self.xkf_stat.message = "Invalid"
		#	if status & 0b0100:
		#		self.gps_stat.level = DiagnosticStatus.OK
		#		self.gps_stat.message = "Ok"
		#	else:
		#		self.gps_stat.level = DiagnosticStatus.WARN
		#		self.gps_stat.message = "No fix"
		#	self.diag_msg.header = h
		#	self.diag_pub.publish(self.diag_msg)

		#	if pub_gps:
		#		if status & 0b0100:
		#			gps_msg.status.status = NavSatStatus.STATUS_FIX
		#			xgps_msg.status.status = GPSStatus.STATUS_FIX
		#			gps_msg.status.service = NavSatStatus.SERVICE_GPS
		#			xgps_msg.status.position_source = 0b01101001
		#			xgps_msg.status.motion_source = 0b01101010
		#			xgps_msg.status.orientation_source = 0b01101010
		#		else:
		#			gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
		#			xgps_msg.status.status = GPSStatus.STATUS_NO_FIX
		#			gps_msg.status.service = 0
		#			xgps_msg.status.position_source = 0b01101000
		#			xgps_msg.status.motion_source = 0b01101000
		#			xgps_msg.status.orientation_source = 0b01101000
		# publish available information
		if pub_imu:
			imu_msg.header = h
			self.imu_pub.publish(imu_msg)
		#if pub_gps:
		#	xgps_msg.header = gps_msg.header = h
		#	self.gps_pub.publish(gps_msg)
		#	self.xgps_pub.publish(xgps_msg)
		if pub_mag:
			mag_msg.header = h
			self.mag_pub.publish(mag_msg)
		#if pub_temp:
		#	self.temp_pub.publish(temp_msg)
		if pub_ss:
			self.ss_pub.publish(ss_msg)
		if pub_ori:
			ori_msg.header = h
			self.ori_pub.publish(ori_msg)
		if pub_vel:
			vel_msg.header = h
			self.vel_pub.publish(vel_msg)
		if pub_pos:
			pos_msg.header = h
			self.pos_pub.publish(pos_msg)		
			

def main():
	'''Create a ROS node and instantiate the class.'''
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	driver.spin()


if __name__== '__main__':
	main()


