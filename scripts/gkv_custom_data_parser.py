#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData

navsat_msg = None
imu_msg = None

def callback(data):
    global navsat_msg, imu_msg

    navsat_msg = NavSatFix()
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'
    navsat_msg.latitude = data.param_values[0]
    navsat_msg.longitude = data.param_values[1]
    navsat_msg.altitude = data.param_values[2]
    navsat_msg.position_covariance = [data.param_values[3], 0, 0,
                                      0, data.param_values[4], 0,
                                      0, 0, data.param_values[5]]

    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'
    imu_msg.orientation.x = data.param_values[6]
    imu_msg.orientation.y = data.param_values[7]
    imu_msg.orientation.z = data.param_values[8]
    imu_msg.orientation.w = data.param_values[9]
    imu_msg.orientation_covariance = [1, 0, 0,
                                      0, 2, 0,
                                      0, 0, 3]
    imu_msg.angular_velocity.x = data.param_values[10]
    imu_msg.angular_velocity.y = data.param_values[11]
    imu_msg.angular_velocity.z = data.param_values[12]
    imu_msg.angular_velocity_covariance = [1, 0, 0,
                                           0, 2, 0,
                                           0, 0, 3]
    imu_msg.linear_acceleration.x = data.param_values[13]
    imu_msg.linear_acceleration.y = data.param_values[14]
    imu_msg.linear_acceleration.z = data.param_values[15]
    imu_msg.linear_acceleration_covariance = [1, 0, 0,
                                              0, 2, 0,
                                              0, 0, 3]

def publish_navsat_msg(_):
    global navsat_msg
    if navsat_msg:
        pub_navsat.publish(navsat_msg)

def publish_imu_msg(_):
    global imu_msg
    if imu_msg:
        pub_imu.publish(imu_msg)

def listener():
    rospy.init_node('gkv_parser', anonymous=True)
    rospy.Subscriber("gkv_custom_data", GkvCustomData, callback)
    global pub_navsat, pub_imu
    pub_navsat = rospy.Publisher('navsatfix_topic', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('imu_topic', Imu, queue_size=10)

    rospy.Timer(rospy.Duration(0.2), publish_navsat_msg) # 5 Гц
    rospy.Timer(rospy.Duration(0.01), publish_imu_msg)   # 100 Гц

    rospy.spin()

if __name__ == '__main__':
    listener()
