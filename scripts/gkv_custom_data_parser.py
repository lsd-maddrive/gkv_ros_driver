#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub_navsat = None
pub_imu = None
pub_odom = None

navsat_msg = NavSatFix()
imu_msg = Imu()
odom_msg = Odometry()

is_ready = False

def navsat_callback(data):
    global navsat_msg
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'

    value = int(data.param_values[3])
    binary_string = format(value, '032b')
    inverted_binary_string = ''.join('1' if bit == '0' else '0' for bit in binary_string)
    bit_17_from_end = int(inverted_binary_string[-18])
    navsat_msg.status.status = bit_17_from_end

    navsat_msg.latitude = data.param_values[4] # 69
    navsat_msg.longitude = data.param_values[5] # 70
    navsat_msg.altitude = data.param_values[6] #71
    navsat_msg.position_covariance = [data.param_values[7], 0, 0, # 85
                                      0, data.param_values[8], 0, # 86
                                      0, 0, data.param_values[9]] # 87
    navsat_msg.position_covariance_type = 2

def imu_callback(data):
    global imu_msg, is_ready
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'

    enu_quaternion = (
        data.param_values[14], # 40
        data.param_values[15], # 41
        data.param_values[16], # 42
        data.param_values[13] # 39
    )
    enu_roll, enu_pitch, enu_yaw = euler_from_quaternion(enu_quaternion)
    # enu to ned
    ned_roll = enu_pitch
    ned_pitch = enu_roll
    ned_yaw = -enu_yaw

    ned_q = quaternion_from_euler(ned_roll, ned_pitch, ned_yaw)
    if ned_q[0] == 0 and ned_q[1] == 0 and ned_q[2] == 0 and ned_q[3] == 1:
        return
    
    if not is_ready:
        rospy.loginfo("GKV is ready!")
        is_ready = True

    imu_msg.orientation.x = ned_q[0]
    imu_msg.orientation.y = ned_q[1]
    imu_msg.orientation.z = ned_q[2]
    imu_msg.orientation.w = ned_q[3]

    imu_msg.orientation_covariance = [data.param_values[18], 0, 0, # 105
                                      0, data.param_values[17], 0, # 106
                                      0, 0, data.param_values[19]] # 104

    imu_msg.angular_velocity.x = data.param_values[21] # 22
    imu_msg.angular_velocity.y = data.param_values[20] # 21
    imu_msg.angular_velocity.z = -data.param_values[22] # 23
    imu_msg.angular_velocity_covariance = [0.094, 0, 0,
                                           0, 0.094, 0,
                                           0, 0, 0.073]

    imu_msg.linear_acceleration.x = data.param_values[33] # 19
    imu_msg.linear_acceleration.y = data.param_values[32] # 18
    imu_msg.linear_acceleration.z = -data.param_values[34] # 20
    imu_msg.linear_acceleration_covariance = [0.009, 0, 0,
                                              0, 0.009, 0,
                                              0, 0, 0.0067]

def odom_callback(data):
    global odom_msg, is_ready
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_footprint'

    odom_msg.pose.pose.position.x = data.param_values[36] # 44
    odom_msg.pose.pose.position.y = data.param_values[35] # 43
    odom_msg.pose.pose.position.z = 0
    # odom_msg.pose.pose.position.z = -data.param_values[37] # 45
    enu_quaternion = (
        data.param_values[14], # 40
        data.param_values[15], # 41
        data.param_values[16], # 42
        data.param_values[13] # 39
    )
    enu_roll, enu_pitch, enu_yaw = euler_from_quaternion(enu_quaternion)
    # enu to ned
    ned_roll = enu_pitch
    ned_pitch = enu_roll
    ned_yaw = -enu_yaw

    ned_q = quaternion_from_euler(ned_roll, ned_pitch, ned_yaw)
    if ned_q[0] == 0 and ned_q[1] == 0 and ned_q[2] == 0 and ned_q[3] == 1:
        return
    
    if not is_ready:
        rospy.loginfo("GKV is ready!")
        is_ready = True

    odom_msg.pose.pose.orientation.x = ned_q[0]
    odom_msg.pose.pose.orientation.y = ned_q[1]
    odom_msg.pose.pose.orientation.z = ned_q[2]
    odom_msg.pose.pose.orientation.w = ned_q[3]
    odom_msg.pose.covariance = [data.param_values[39], 0, 0, 0, 0, 0, # 99
                                0, data.param_values[38], 0, 0, 0, 0, # 98
                                0, 0, data.param_values[40], 0, 0, 0, # 100
                                0, 0, 0, data.param_values[18], 0, 0, # 105
                                0, 0, 0, 0, data.param_values[17], 0, # 106
                                0, 0, 0, 0, 0, data.param_values[19]] # 104

    odom_msg.twist.twist.linear.x = data.param_values[24] # 47
    odom_msg.twist.twist.linear.y = data.param_values[23] # 46
    odom_msg.twist.twist.linear.z = -data.param_values[25] # 48
    odom_msg.twist.twist.angular.x = data.param_values[21] # 22
    odom_msg.twist.twist.angular.y = data.param_values[20] # 21
    odom_msg.twist.twist.angular.z = -data.param_values[22] # 23
    odom_msg.twist.covariance = [data.param_values[27], 0, 0, 0, 0, 0, # 102
                                 0, data.param_values[26], 0, 0, 0, 0, # 101
                                 0, 0, data.param_values[28], 0, 0, 0, # 103
                                 0, 0, 0, 0.094, 0, 0,
                                 0, 0, 0, 0, 0.094, 0,
                                 0, 0, 0, 0, 0, 0.073]

def publish_navsat(_):
    pub_navsat.publish(navsat_msg)

def publish_imu(_):
    pub_imu.publish(imu_msg)

def publish_odom(_):
    pub_odom.publish(odom_msg)

def listener():
    rospy.init_node('gkv_parser', anonymous=True)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, navsat_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, imu_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, odom_callback, queue_size=1)

    global pub_navsat, pub_imu, pub_odom
    pub_navsat = rospy.Publisher('gkv/navsat/fix', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('gkv/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('gkv/odom', Odometry, queue_size=10)

    rospy.Timer(rospy.Duration(0.1), publish_navsat) # 10 Гц
    rospy.Timer(rospy.Duration(0.01), publish_imu) # 100 Гц
    rospy.Timer(rospy.Duration(0.05), publish_odom) # 20 Гц

    rospy.spin()

if __name__ == '__main__':
    listener()
