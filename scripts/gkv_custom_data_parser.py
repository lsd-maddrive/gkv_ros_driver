#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub_navsat = None
pub_imu = None
pub_odom = None

# Publish frequency parameters
navsat_freq = None
imu_freq = None
odom_freq = None

def navsat_callback(data):
    navsat_msg = NavSatFix()
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'

    value = int(data.param_values[3])
    binary_string = format(value, '032b')
    inverted_binary_string = ''.join('1' if bit == '0' else '0' for bit in binary_string)
    bit_17_from_end = int(inverted_binary_string[-18])
    navsat_msg.status.status = bit_17_from_end

    navsat_msg.latitude = data.param_values[4] # 69
    navsat_msg.longitude = data.param_values[5] # 70
    navsat_msg.altitude = data.param_values[6] # 71
    navsat_msg.position_covariance = [data.param_values[7], 0, 0, # 85
                                      0, data.param_values[8], 0, # 86
                                      0, 0, data.param_values[9]] # 87
    navsat_msg.position_covariance_type = 2

    pub_navsat.publish(navsat_msg)

def imu_callback(data):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'

    quaternion = (
        data.param_values[14], # 39
        data.param_values[15], # 40
        data.param_values[16], # 41
        data.param_values[13] # 42
    )
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    roll_shifted = pitch
    pitch_shifted = roll
    yaw_shifted = -yaw
    q = quaternion_from_euler(roll_shifted, pitch_shifted, yaw_shifted)
    imu_msg.orientation.x = q[0]
    imu_msg.orientation.y = q[1]
    imu_msg.orientation.z = q[2]
    imu_msg.orientation.w = q[3]

    # imu_msg.orientation.x = data.param_values[14] # 39
    # imu_msg.orientation.y = data.param_values[15] # 40
    # imu_msg.orientation.z = data.param_values[16] # 41
    # imu_msg.orientation.w = data.param_values[13] # 42

    imu_msg.orientation_covariance = [data.param_values[17], 0, 0, # 106
                                      0, data.param_values[18], 0, # 105
                                      0, 0, data.param_values[19]] # 104

    imu_msg.angular_velocity.x = data.param_values[21] # 21
    imu_msg.angular_velocity.y = data.param_values[20] # 22
    imu_msg.angular_velocity.z = data.param_values[22] # 23
    imu_msg.angular_velocity_covariance = [0.094, 0, 0,
                                           0, 0.094, 0,
                                           0, 0, 0.073]

    imu_msg.linear_acceleration.x = data.param_values[33] # 18 или 49
    imu_msg.linear_acceleration.y = data.param_values[32] # 19 или 50
    imu_msg.linear_acceleration.z = data.param_values[34] # 20 или 51
    imu_msg.linear_acceleration_covariance = [0.009, 0, 0,
                                              0, 0.009, 0,
                                              0, 0, 0.0067]

    pub_imu.publish(imu_msg)

def odom_callback(data):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_footprint'

    odom_msg.pose.pose.position.x = data.param_values[36] # 43
    odom_msg.pose.pose.position.y = data.param_values[35] # 44
    odom_msg.pose.pose.position.z = -data.param_values[37] # 45
    # odom_msg.pose.pose.orientation.x = data.param_values[14] # 40
    # odom_msg.pose.pose.orientation.y = data.param_values[15] # 41
    # odom_msg.pose.pose.orientation.z = data.param_values[16] # 42
    # odom_msg.pose.pose.orientation.w = data.param_values[13] # 39
    quaternion = (
        data.param_values[14], # 39
        data.param_values[15], # 40
        data.param_values[16], # 41
        data.param_values[13] # 42
    )
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    roll_shifted = pitch
    pitch_shifted = roll
    yaw_shifted = -yaw
    q = quaternion_from_euler(roll_shifted, pitch_shifted, yaw_shifted)
    if q[0] == 0:
        print(q)
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]
    odom_msg.pose.covariance = [data.param_values[38], 0, 0, 0, 0, 0, # 98
                                0, data.param_values[39], 0, 0, 0, 0, # 99
                                0, 0, data.param_values[40], 0, 0, 0, # 100
                                0, 0, 0, data.param_values[17], 0, 0, # 106
                                0, 0, 0, 0, data.param_values[18], 0, # 105
                                0, 0, 0, 0, 0, data.param_values[19]] # 104

    odom_msg.twist.twist.linear.x = data.param_values[24] # 46
    odom_msg.twist.twist.linear.y = data.param_values[23] # 47
    odom_msg.twist.twist.linear.z = -data.param_values[25] # 48
    odom_msg.twist.twist.angular.x = data.param_values[21] # 21
    odom_msg.twist.twist.angular.y = data.param_values[20] # 22
    odom_msg.twist.twist.angular.z = -data.param_values[22] # 23
    odom_msg.twist.covariance = [data.param_values[26], 0, 0, 0, 0, 0, # 101
                                 0, data.param_values[27], 0, 0, 0, 0, # 102
                                 0, 0, data.param_values[28], 0, 0, 0, # 103
                                 0, 0, 0, 0.094, 0, 0,
                                 0, 0, 0, 0, 0.094, 0,
                                 0, 0, 0, 0, 0, 0.073]

    pub_odom.publish(odom_msg)

def setup_timers():
    global navsat_freq, imu_freq, odom_freq
    navsat_freq = rospy.get_param('~navsat_freq', 10.0)  # Default frequency is 10 Hz
    imu_freq = rospy.get_param('~imu_freq', 100.0)  # Default frequency is 100 Hz
    odom_freq = rospy.get_param('~odom_freq', 50.0)  # Default frequency is 50 Hz

    rospy.Timer(rospy.Duration(1.0 / navsat_freq), lambda event: pub_navsat.publish(navsat_msg))
    rospy.Timer(rospy.Duration(1.0 / imu_freq), lambda event: pub_imu.publish(imu_msg))
    rospy.Timer(rospy.Duration(1.0 / odom_freq), lambda event: pub_odom.publish(odom_msg))

def listener():
    rospy.init_node('gkv_parser', anonymous=True)

    global pub_navsat, pub_imu, pub_odom
    pub_navsat = rospy.Publisher('gkv/navsat/fix', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('gkv/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('gkv/odom', Odometry, queue_size=10)

    setup_timers()

    rospy.Subscriber('gkv_custom_data', GkvCustomData, navsat_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, imu_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, odom_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
