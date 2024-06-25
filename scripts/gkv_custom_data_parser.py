#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

navsat_msg = None
imu_msg = None
odom_msg = None

def callback(data):
    global navsat_msg, imu_msg, odom_msg

    # navsat fix msg
    navsat_msg = NavSatFix()
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'

    # navsat_msg.status.status = data.param_values[3] # 72
    navsat_msg.latitude = data.param_values[4] # 69
    navsat_msg.longitude = data.param_values[5] # 70
    navsat_msg.altitude = data.param_values[6] #71
    navsat_msg.position_covariance = [data.param_values[7], 0, 0, # 85
                                      0, data.param_values[8], 0, # 86
                                      0, 0, data.param_values[9]] # 87
    navsat_msg.position_covariance_type = 2

    # imu msg
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'

    imu_msg.orientation.x = data.param_values[14] # 40
    imu_msg.orientation.y = data.param_values[15] # 41
    imu_msg.orientation.z = data.param_values[16] # 42
    imu_msg.orientation.w = data.param_values[13] # 39

    imu_msg.orientation_covariance = [data.param_values[17], 0, 0, # 106
                                      0, data.param_values[18], 0, # 105
                                      0, 0, data.param_values[19]] # 104

    imu_msg.angular_velocity.x = data.param_values[20] # 21
    imu_msg.angular_velocity.y = data.param_values[21] # 22
    imu_msg.angular_velocity.z = data.param_values[22] # 23
    imu_msg.angular_velocity_covariance = [0.094, 0, 0,
                                           0, 0.094, 0,
                                           0, 0, 0.073]

    imu_msg.linear_acceleration.x = data.param_values[32] # 18 или 49
    imu_msg.linear_acceleration.y = data.param_values[33] # 19 или 50
    imu_msg.linear_acceleration.z = data.param_values[34] # 20 или 51
    imu_msg.linear_acceleration_covariance = [0.009, 0, 0,
                                              0, 0.009, 0,
                                              0, 0, 0.0067]

    # odom msg
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'gkv_alg_link'

    odom_msg.pose.pose.position.x = data.param_values[35] # 43
    odom_msg.pose.pose.position.y = data.param_values[36] # 44
    odom_msg.pose.pose.position.z = data.param_values[37] # 45
    odom_msg.pose.pose.orientation.x = data.param_values[14] # 40
    odom_msg.pose.pose.orientation.y = data.param_values[15] # 41
    odom_msg.pose.pose.orientation.z = data.param_values[16] # 42
    odom_msg.pose.pose.orientation.w = data.param_values[13] # 39
    odom_msg.pose.covariance = [data.param_values[38], 0, 0, 0, 0, 0, # 98
                                0, data.param_values[39], 0, 0, 0, 0, # 99
                                0, 0, data.param_values[40], 0, 0, 0, # 100
                                0, 0, 0, data.param_values[17], 0, 0, # 106
                                0, 0, 0, 0, data.param_values[18], 0, # 105
                                0, 0, 0, 0, 0, data.param_values[19]] # 104

    odom_msg.twist.twist.linear.x = data.param_values[23] # 46
    odom_msg.twist.twist.linear.y = data.param_values[24] # 47
    odom_msg.twist.twist.linear.z = data.param_values[25] # 48
    odom_msg.twist.twist.angular.x = data.param_values[20] # 21
    odom_msg.twist.twist.angular.y = data.param_values[21] # 22
    odom_msg.twist.twist.angular.z = data.param_values[22] # 23
    odom_msg.twist.covariance = [data.param_values[26], 0, 0, 0, 0, 0, # 101
                                 0, data.param_values[27], 0, 0, 0, 0, # 102
                                 0, 0, data.param_values[28], 0, 0, 0, # 103
                                 0, 0, 0, 0.094, 0, 0,
                                 0, 0, 0, 0, 0.094, 0,
                                 0, 0, 0, 0, 0, 0.073]

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
    rospy.Subscriber('gkv_custom_data', GkvCustomData, callback)
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
