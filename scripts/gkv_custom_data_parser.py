#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu, NavSatFix
from gkv_ros_driver.msg import GkvCustomData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from decimal import Decimal
import tf
from geometry_msgs.msg import  PoseStamped


pub_navsat = None
pub_imu = None
pub_odom = None
pub = None

navsat_msg = NavSatFix()
imu_msg = Imu()
odom_msg = Odometry()

# flag if GKV algo is stage 50
is_ready = False

initial_x = None
initial_y = None

def navsat_callback(data):
    global navsat_msg
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.header.frame_id = 'gkv_gnss_master_link'

    value = int(data.param_values[3])
    binary_string = format(value, '032b')
    inverted_binary_string = ''.join('1' if bit == '0' else '0' for bit in binary_string)
    bit_17_from_end = int(inverted_binary_string[-18])
    navsat_msg.status.status = bit_17_from_end

    alg_lat = data.param_values[35] * (2 * 3.14159265359 / 2 ** 32) * (180 / 3.14159265359) # 91 alg_int_lat (alg)
    alg_lon = data.param_values[36] * (2 * 3.14159265359 / 2 ** 32) * (180 / 3.14159265359) # 92 alg_int_lon (alg)
    alg_alt = data.param_values[37] # 93 alg_alt (alg)

    navsat_msg.latitude = alg_lat
    navsat_msg.longitude = alg_lon
    navsat_msg.altitude = alg_alt
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
    global odom_msg, is_ready, initial_x, initial_y

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'odom'

    odom_msg.child_frame_id = 'base_footprint'

    R = 6371100
    x_gnss_rad = data.param_values[36] * (2 * math.pi / 2 ** 32) # 92 alg_int_lon (alg)
    y_gnss_rad = data.param_values[35] * (2 * math.pi / 2 ** 32) # 91 alg_int_lat (alg)

    # print('FIRST COORDINATE %f , %f', x_gnss_rad, y_gnss_rad)
    X_GNSS_INIT = 0.9075741764045073
    Y_GNSS_INIT = 0.9742335103819032
    # AZIMUTH_INIT = -1.54768047862242
    X_GNSS_INIT_COS = Decimal(math.cos(Decimal(Y_GNSS_INIT))) 

    x_ned = Decimal(R * (Decimal(x_gnss_rad) - Decimal(X_GNSS_INIT)) * X_GNSS_INIT_COS) * Decimal(1.0)
    # Decimal(0.9309686727)
    y_ned = Decimal(R * (Decimal(y_gnss_rad) - Decimal(Y_GNSS_INIT))) * Decimal(1.0)
    # Decimal(1.0171345)
    # print('My new y %f in radians is %f new y is %f odometry value is %f', data.param_values[3], y_gnss_rad, R * (y_gnss_rad ) , y)
    # pose_stamped = PoseStamped()
    # pose_stamped.header.stamp = rospy.Time.now()
    # pose_stamped.header.frame_id = "odom"
    # pose_stamped.pose.position.x = x
    # pose_stamped.pose.position.y = y
    # pose_stamped.pose.position.z = 0.0
    # pose_stamped.pose.orientation.w = 1.0

    # rospy.logdebug("Publishing PoseStamped message: %s", pose_stamped)

    odom_msg.pose.pose.position.x = x_ned
    odom_msg.pose.pose.position.y = y_ned
    # odom_msg.pose.pose.position.x = x_ned * Decimal(math.cos(AZIMUTH_INIT)) + y_ned * Decimal(math.sin(AZIMUTH_INIT))
    # odom_msg.pose.pose.position.y = -x_ned * Decimal(math.sin(AZIMUTH_INIT)) + y_ned * Decimal(math.cos(AZIMUTH_INIT))

    # odom_msg.pose.pose.position.z = - data.param_values[37]
    odom_msg.pose.pose.position.z = 0

    enu_quaternion = (
        data.param_values[14], # 40 q1 (alg)
        data.param_values[15], # 41 q2 (alg)
        data.param_values[16], # 42 q3 (alg)
        data.param_values[13] # 39 q0 (alg)
    )
    enu_roll, enu_pitch, enu_yaw = euler_from_quaternion(enu_quaternion)
    # enu to ned
    ned_roll = 0
    ned_pitch = 0
    # ned_roll = enu_pitch
    # ned_pitch = enu_roll
    ned_yaw = -enu_yaw
    # ned_yaw = -enu_yaw - (AZIMUTH_INIT)

    ned_q = quaternion_from_euler(ned_roll, ned_pitch, ned_yaw)
    if ned_q[0] == 0 and ned_q[1] == 0 and ned_q[2] == 0 and ned_q[3] == 1:
        return
    # broadcast tf odom->base_footprint
    # tf_broadcaster = tf.TransformBroadcaster()
    # tf_broadcaster.sendTransform(
    #     (x, y, 0.0),
    #     (ned_q[0], ned_q[1], ned_q[2], ned_q[3]),
    #     rospy.Time.now(),
    #     "base_footprint",
    #     "odom"
    # )
    
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

def publish_navsat(_):
    pub_navsat.publish(navsat_msg)

def publish_imu(_):
    if is_ready:
        pub_imu.publish(imu_msg)

def publish_odom(_):
    if is_ready:
        pub_odom.publish(odom_msg)

def listener():
    rospy.init_node('gkv_parser', anonymous=False)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, navsat_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, imu_callback, queue_size=1)
    rospy.Subscriber('gkv_custom_data', GkvCustomData, odom_callback, queue_size=1)

    global pub_navsat, pub_imu, pub_odom, pub
    pub_navsat = rospy.Publisher('gkv/navsat/fix', NavSatFix, queue_size=10)
    pub_imu = rospy.Publisher('gkv/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('gkv/odom', Odometry, queue_size=10)
    # pub = rospy.Publisher('/gps_transformation', PoseStamped, queue_size=10)

    rospy.Timer(rospy.Duration(0.1), publish_navsat) # 10 Гц
    rospy.Timer(rospy.Duration(0.01), publish_imu) # 100 Гц
    rospy.Timer(rospy.Duration(0.05), publish_odom) # 20 Гц

    rospy.spin()

if __name__ == '__main__':
    listener()
