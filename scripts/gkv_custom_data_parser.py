#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub_navsat = None
pub_imu = None
pub_odom = None
pub_geo_odom = None

navsat_msg = NavSatFix()
imu_msg = Imu()
odom_msg = Odometry()
geo_odom_msg = Odometry()

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

    navsat_msg.latitude = data.param_values[4] # 69 gnss_latitude (gnss)
    navsat_msg.longitude = data.param_values[5] # 70 gnss_longitude (gnss)
    navsat_msg.altitude = data.param_values[6] # 71 gnss_altitude (gnss)
    navsat_msg.position_covariance = [data.param_values[7], 0, 0, # 85 gnss_sig_lat (gnss)
                                      0, data.param_values[8], 0, # 86 gnss_sig_lon (gnss)
                                      0, 0, data.param_values[9]] # 87 gnss_sig_alt (gnss)
    navsat_msg.position_covariance_type = 2

def imu_callback(data):
    global imu_msg, is_ready
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gkv_imu_link'

    enu_quaternion = (
        data.param_values[14], # 40 q1 (alg)
        data.param_values[15], # 41 q2 (alg)
        data.param_values[16], # 42 q3 (alg)
        data.param_values[13] # 39 q0 (alg)
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

    imu_msg.orientation_covariance = [data.param_values[18], 0, 0, # 105 alg_var_theta (pitch) (alg)
                                      0, data.param_values[17], 0, # 106 alg_var_phi (roll) (alg)
                                      0, 0, data.param_values[19]] # 104 alg_var_psi (yaw) (alg)

    imu_msg.angular_velocity.x = data.param_values[21] # 22 wy (sensor)
    imu_msg.angular_velocity.y = data.param_values[20] # 21 wx (sensor)
    imu_msg.angular_velocity.z = -data.param_values[22] # 23 wz (sensor)
    imu_msg.angular_velocity_covariance = [0.094, 0, 0,
                                           0, 0.094, 0,
                                           0, 0, 0.073]

    imu_msg.linear_acceleration.x = data.param_values[33] # 19 ay (sensor)
    imu_msg.linear_acceleration.y = data.param_values[32] # 18 ax (sensor)
    imu_msg.linear_acceleration.z = -data.param_values[34] # 20 az (sensor)
    imu_msg.linear_acceleration_covariance = [0.009, 0, 0,
                                              0, 0.009, 0,
                                              0, 0, 0.0067]

def odom_callback(data):
    global odom_msg, is_ready
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_footprint'

    odom_msg.pose.pose.position.x = data.param_values[36] # 44 y (alg)
    odom_msg.pose.pose.position.y = data.param_values[35] # 43 x (alg)
    odom_msg.pose.pose.position.z = 0
    # odom_msg.pose.pose.position.z = -data.param_values[37] # 45 z (alg)
    enu_quaternion = (
        data.param_values[14], # 40 q1 (alg)
        data.param_values[15], # 41 q2 (alg)
        data.param_values[16], # 42 q3 (alg)
        data.param_values[13] # 39 q0 (alg)
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
    odom_msg.pose.covariance = [data.param_values[39], 0, 0, 0, 0, 0, # 99 alg_var_y (alg)
                                0, data.param_values[38], 0, 0, 0, 0, # 98 alg_var_x (alg)
                                0, 0, data.param_values[40], 0, 0, 0, # 100 alg_var_z (alg)
                                0, 0, 0, data.param_values[18], 0, 0, # 105 alg_var_theta (pitch) (alg)
                                0, 0, 0, 0, data.param_values[17], 0, # 106 alg_var_phi (roll) (alg)
                                0, 0, 0, 0, 0, data.param_values[19]] # 104 alg_var_psi (yaw) (alg)

    odom_msg.twist.twist.linear.x = data.param_values[24] # 47 vy (alg)
    odom_msg.twist.twist.linear.y = data.param_values[23] # 46 vx (alg)
    odom_msg.twist.twist.linear.z = -data.param_values[25] # 48 vz (alg)
    odom_msg.twist.twist.angular.x = data.param_values[21] # 22 wy (sensor)
    odom_msg.twist.twist.angular.y = data.param_values[20] # 21 wx (sensor)
    odom_msg.twist.twist.angular.z = -data.param_values[22] # 23 wz (sensor)
    odom_msg.twist.covariance = [data.param_values[27], 0, 0, 0, 0, 0, # 102 alg_var_vy (alg)
                                 0, data.param_values[26], 0, 0, 0, 0, # 101 alg_var_vx (alg)
                                 0, 0, data.param_values[28], 0, 0, 0, # 103 alg_var_vz (alg)
                                 0, 0, 0, 0.094, 0, 0,
                                 0, 0, 0, 0, 0.094, 0,
                                 0, 0, 0, 0, 0, 0.073]

def geo_odom_callback(data):
    global geo_odom_msg, is_ready
    geo_odom_msg.header.stamp = rospy.Time.now()

    geo_odom_msg.pose.pose.position.x = data.param_values[4] # 69 gnss_latitude (gnss)
    geo_odom_msg.pose.pose.position.y = data.param_values[5] # 70 gnss_longitude (gnss)
    geo_odom_msg.pose.pose.position.z = data.param_values[6] # 71 gnss_altitude (gnss)
    enu_quaternion = (
        data.param_values[14], # 40 q1 (alg)
        data.param_values[15], # 41 q2 (alg)
        data.param_values[16], # 42 q3 (alg)
        data.param_values[13] # 39 q0 (alg)
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

    geo_odom_msg.pose.pose.orientation.x = ned_q[0]
    geo_odom_msg.pose.pose.orientation.y = ned_q[1]
    geo_odom_msg.pose.pose.orientation.z = ned_q[2]
    geo_odom_msg.pose.pose.orientation.w = ned_q[3]
    geo_odom_msg.pose.covariance = [data.param_values[7], 0, 0, 0, 0, 0, # 85 gnss_sig_lat (gnss)
                                0, data.param_values[8], 0, 0, 0, 0, # 86 gnss_sig_lon (gnss)
                                0, 0, data.param_values[9], 0, 0, 0, # 87 gnss_sig_alt (gnss)
                                0, 0, 0, data.param_values[18], 0, 0, # 105 alg_var_theta (pitch) (alg)
                                0, 0, 0, 0, data.param_values[17], 0, # 106 alg_var_phi (roll) (alg)
                                0, 0, 0, 0, 0, data.param_values[19]] # 104 alg_var_psi (yaw) (alg)

    geo_odom_msg.twist.twist.linear.x = data.param_values[24] # 47 vy (alg)
    geo_odom_msg.twist.twist.linear.y = data.param_values[23] # 46 vx (alg)
    geo_odom_msg.twist.twist.linear.z = -data.param_values[25] # 48 vz (alg)
    geo_odom_msg.twist.twist.angular.x = data.param_values[21] # 22 wy (sensor)
    geo_odom_msg.twist.twist.angular.y = data.param_values[20] # 21 wx (sensor)
    geo_odom_msg.twist.twist.angular.z = -data.param_values[22] # 23 wz (sensor)
    geo_odom_msg.twist.covariance = [data.param_values[27], 0, 0, 0, 0, 0, # 102 alg_var_vy (alg)
                                 0, data.param_values[26], 0, 0, 0, 0, # 101 alg_var_vx (alg)
                                 0, 0, data.param_values[28], 0, 0, 0, # 103 alg_var_vz (alg)
                                 0, 0, 0, 0.094, 0, 0,
                                 0, 0, 0, 0, 0.094, 0,
                                 0, 0, 0, 0, 0, 0.073]

def publish_navsat(_):
    pub_navsat.publish(navsat_msg)

def publish_imu(_):
    if is_ready:
        pub_imu.publish(imu_msg)

def publish_odom(_):
    if is_ready:
        pub_odom.publish(odom_msg)

def publish_geo_odom(_):
    if is_ready:
        pub_geo_odom.publish(geo_odom_msg)

def listener():
    rospy.init_node('gkv_parser', anonymous=True)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, navsat_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, imu_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, odom_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, geo_odom_callback, queue_size=1)

    global pub_navsat, pub_imu, pub_odom, pub_geo_odom
    pub_navsat = rospy.Publisher('gkv/navsat/fix', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('gkv/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('gkv/odom', Odometry, queue_size=10)
    pub_geo_odom = rospy.Publisher('gkv/geo_odom', Odometry, queue_size=10)

    rospy.Timer(rospy.Duration(0.1), publish_navsat) # 10 Гц
    rospy.Timer(rospy.Duration(0.01), publish_imu) # 100 Гц
    rospy.Timer(rospy.Duration(0.05), publish_odom) # 20 Гц
    rospy.Timer(rospy.Duration(0.05), publish_geo_odom) # 20 Гц

    rospy.spin()

if __name__ == '__main__':
    listener()
