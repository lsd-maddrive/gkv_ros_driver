#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry

navsat_msg = None
imu_msg = None
odom_msg = None

def callback(data):
    global navsat_msg, imu_msg, odom_msg

    # navsat fix msg
    navsat_msg = NavSatFix()
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'
    navsat_msg.latitude = data.param_values[x]
    navsat_msg.longitude = data.param_values[x]
    navsat_msg.altitude = data.param_values[x]
    navsat_msg.position_covariance = [data.param_values[x], 0, 0,
                                      0, data.param_values[x], 0,
                                      0, 0, data.param_values[x]]

    # imu msg
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'
    imu_msg.orientation.x = data.param_values[x]
    imu_msg.orientation.y = data.param_values[x]
    imu_msg.orientation.z = data.param_values[x]
    imu_msg.orientation.w = data.param_values[x]
    imu_msg.orientation_covariance = [data.param_values[x], 0, 0,
                                      0, data.param_values[x], 0,
                                      0, 0, data.param_values[x]]
    imu_msg.angular_velocity.x = data.param_values[x]
    imu_msg.angular_velocity.y = data.param_values[x]
    imu_msg.angular_velocity.z = data.param_values[x]
    imu_msg.angular_velocity_covariance = [data.param_values[x], 0, 0,
                                           0, data.param_values[x], 0,
                                           0, 0, data.param_values[x]]
    imu_msg.linear_acceleration.x = data.param_values[x]
    imu_msg.linear_acceleration.y = data.param_values[x]
    imu_msg.linear_acceleration.z = data.param_values[x]
    imu_msg.linear_acceleration_covariance = [data.param_values[x], 0, 0,
                                              0, data.param_values[x], 0,
                                              0, 0, data.param_values[x]]

    # odom msg
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'gkv_alg_link'
    odom_msg.pose.pose.position.x = data.param_values[x]
    odom_msg.pose.pose.position.y = data.param_values[x]
    odom_msg.pose.pose.position.z = data.param_values[x]
    odom_msg.pose.pose.orientation.x = data.param_values[x]
    odom_msg.pose.pose.orientation.y = data.param_values[x]
    odom_msg.pose.pose.orientation.z = data.param_values[x]
    odom_msg.pose.pose.orientation.w = data.param_values[x]
    odom_msg.pose.covariance = [data.param_values[x], 0, 0, 0, 0, 0,
                                0, data.param_values[x], 0, 0, 0, 0,
                                0, 0, data.param_values[x], 0, 0, 0,
                                0, 0, 0, data.param_values[x], 0, 0,
                                0, 0, 0, 0, data.param_values[x], 0,
                                0, 0, 0, 0, 0, data.param_values[x]]
    odom_msg.twist.twist.linear.x = data.param_values[x]
    odom_msg.twist.twist.linear.y = data.param_values[x]
    odom_msg.twist.twist.linear.z = data.param_values[x]
    odom_msg.twist.twist.angular.x = data.param_values[x]
    odom_msg.twist.twist.angular.y = data.param_values[x]
    odom_msg.twist.twist.angular.z = data.param_values[x]
    odom_msg.twist.covariance = [data.param_values[x], 0, 0, 0, 0, 0,
                                 0, data.param_values[x], 0, 0, 0, 0,
                                 0, 0, data.param_values[x], 0, 0, 0,
                                 0, 0, 0, data.param_values[x], 0, 0,
                                 0, 0, 0, 0, data.param_values[x], 0,
                                 0, 0, 0, 0, 0, data.param_values[x]]

def publish_navsat_msg(_):
    global navsat_msg
    if navsat_msg:
        pub_navsat.publish(navsat_msg)

def publish_imu_msg(_):
    global imu_msg
    if imu_msg:
        pub_imu.publish(imu_msg)

def publish_odom_msg(_):
    global odom_msg
    if odom_msg:
        pub_odom.publish(odom_msg)

def listener():
    rospy.init_node('gkv_parser', anonymous=True)
    rospy.Subscriber("gkv_custom_data", GkvCustomData, callback)
    global pub_navsat, pub_imu, pub_odom
    pub_navsat = rospy.Publisher('gkv/navsat/fix', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('gkv/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('gkv/odom', Odometry, queue_size=10)

    rospy.Timer(rospy.Duration(0.2), publish_navsat_msg) # 5 Гц
    rospy.Timer(rospy.Duration(0.01), publish_imu_msg)   # 100 Гц
    rospy.Timer(rospy.Duration(0.02), publish_odom_msg)   # 50 Гц

    rospy.spin()

if __name__ == '__main__':
    listener()
