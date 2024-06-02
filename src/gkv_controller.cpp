#include "ros/ros.h"
#include "GKV_DeviceROSWrapper.h"


int main(int argc, char **argv)
{
    std::string com_port;
    // std::cout << "Set Serial Port:";
    // std::cin >> com_port;
    com_port = "/dev/ttyUSB0";
    std::cout << "#start connecting to " << com_port << "\n";
    ros::init(argc, argv, "gkv_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    GKV_DeviceROSWrapper * gkv= new GKV_DeviceROSWrapper(&nh,com_port,B3000000);
    // GKV_DeviceROSWrapper * gkv= new GKV_DeviceROSWrapper(&nh,com_port,B3000000,GKV_LMP_PACKET_MODE);

    if (gkv->IsConnected())
    {
        ROS_INFO("GKV driver is now started");
    }
    else {
        ROS_ERROR("Failed to connect device");
        return 0;
    }
    ros::waitForShutdown();
}
// %EndTag(FULLTEXT)%
