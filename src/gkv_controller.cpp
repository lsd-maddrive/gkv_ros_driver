#include "ros/ros.h"
#include "GKV_DeviceROSWrapper.h"


int main(int argc, char **argv)
{
    std::string com_port = "/dev/ttyGKV-3-main";
    std::cout << "#start connecting to " << com_port << "\n";
    ros::init(argc, argv, "gkv_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    GKV_DeviceROSWrapper * gkv= new GKV_DeviceROSWrapper(&nh,com_port,B921600);
    if (gkv->IsConnected())
    {
        ROS_INFO("GKV driver is now started");
    }
    else {
        ROS_ERROR("Failed to connect device");
        return 0;
    }
    // DEFINE CLIENT FOR PACKET TYPE SETTING
    ros::ServiceClient set_packet_type_client = nh.serviceClient<gkv_ros_driver::GkvSetPacketType>("gkv_set_packet_type_srv");
    gkv_ros_driver::GkvSetPacketType set_packet_type_srv;
    set_packet_type_srv.request.packet_type = 1;
     if (set_packet_type_client.call(set_packet_type_srv))
     {
       ROS_INFO("Packet type changed: %d", (int)set_packet_type_srv.response.result);
     }
     else
     {
       ROS_ERROR("Failed to set packet type");
     }
     // DEFINE CLIENT FOR RESET SERVICE
    ros::ServiceClient reset_client = nh.serviceClient<gkv_ros_driver::GkvReset>("gkv_reset_srv");
    gkv_ros_driver::GkvReset reset_srv;
    if (reset_client.call(reset_srv))
    {
            ROS_INFO("Reseted: %d", (int)reset_srv.response.result);
    }
    else
    {
            ROS_ERROR("Failed to reset device");
    }

    ros::waitForShutdown();
}
// %EndTag(FULLTEXT)%
