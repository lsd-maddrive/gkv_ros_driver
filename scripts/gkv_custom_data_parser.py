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

    # navsat_msg.status.status = data.param_values[1] # 72
    navsat_msg.latitude = data.param_values[15] # 69
    navsat_msg.longitude = data.param_values[16] # 70
    navsat_msg.altitude = data.param_values[17] #71
    navsat_msg.position_covariance = [0.0141, 0, 0, # 85
                                      0, 0.0141, 0, # 86
                                      0, 0, 0.0141] # 87
    navsat_msg.position_covariance_type = 2

    # imu msg
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'

    # quaternion = (
    #     data.param_values[11], # 39
    #     data.param_values[12], # 40
    #     data.param_values[13], # 41
    #     data.param_values[14] # 42
    # )
    # roll, pitch, yaw = euler_from_quaternion(quaternion)
    # roll_shifted = yaw + 0
    # pitch_shifted = pitch + 0
    # yaw_shifted = roll+ 0
    # q = quaternion_from_euler(roll_shifted, pitch_shifted, yaw_shifted)
    # imu_msg.orientation.x = q[0]
    # imu_msg.orientation.y = q[1]
    # imu_msg.orientation.z = q[2]
    # imu_msg.orientation.w = q[3]

    imu_msg.orientation.x = data.param_values[11] # 39
    imu_msg.orientation.y = data.param_values[12] # 40
    imu_msg.orientation.z = data.param_values[13] # 41
    imu_msg.orientation.w = data.param_values[14] # 42

    imu_msg.orientation_covariance = [data.param_values[29], 0, 0, # 104
                                      0, data.param_values[30], 0, # 105
                                      0, 0, data.param_values[31]] # 106

    imu_msg.angular_velocity.x = data.param_values[5] # 21
    imu_msg.angular_velocity.y = data.param_values[6] # 22
    imu_msg.angular_velocity.z = data.param_values[7] # 23
    # imu_msg.angular_velocity_covariance = [data.param_values[], 0, 0,
    #                                        0, data.param_values[], 0,
    #                                        0, 0, data.param_values[]]

    # imu_msg.linear_acceleration.x = data.param_values[] # 64
    # imu_msg.linear_acceleration.y = data.param_values[] # 65
    # imu_msg.linear_acceleration.z = data.param_values[] # 66
    # imu_msg.linear_acceleration_covariance = [data.param_values[], 0, 0,
    #                                           0, data.param_values[], 0,
    #                                           0, 0, data.param_values[]]

    # odom msg
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'gkv_alg_link'

    # odom_msg.pose.pose.position.x = data.param_values[] # 43
    # odom_msg.pose.pose.position.y = data.param_values[] # 44
    # odom_msg.pose.pose.position.z = data.param_values[] # 45
    odom_msg.pose.pose.orientation.x = data.param_values[11] # 39
    odom_msg.pose.pose.orientation.y = data.param_values[12] # 40 
    odom_msg.pose.pose.orientation.z = data.param_values[13] # 41
    odom_msg.pose.pose.orientation.w = data.param_values[14] # 42
    odom_msg.pose.covariance = [data.param_values[23], 0, 0, 0, 0, 0, # 98
                                0, data.param_values[24], 0, 0, 0, 0, # 99
                                0, 0, data.param_values[25], 0, 0, 0, # 100
                                0, 0, 0, data.param_values[29], 0, 0, # 104
                                0, 0, 0, 0, data.param_values[30], 0, # 105
                                0, 0, 0, 0, 0, data.param_values[31]] # 106

    odom_msg.twist.twist.linear.x = data.param_values[20] # 46
    odom_msg.twist.twist.linear.y = data.param_values[21] # 47
    odom_msg.twist.twist.linear.z = data.param_values[22] # 48
    odom_msg.twist.twist.angular.x = data.param_values[5] # 21
    odom_msg.twist.twist.angular.y = data.param_values[6] # 22
    odom_msg.twist.twist.angular.z = data.param_values[7] # 23
    odom_msg.twist.covariance = [data.param_values[26], 0, 0, 0, 0, 0, # 101
                                 0, data.param_values[27], 0, 0, 0, 0, # 102
                                 0, 0, data.param_values[28], 0, 0, 0, # 103
                                 0, 0, 0, data.param_values[5], 0, 0, # 21
                                 0, 0, 0, 0, data.param_values[6], 0, # 22
                                 0, 0, 0, 0, 0, data.param_values[7]] # 23

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
