/**
  ******************************************************************************
  * @file    GKV_DeviceROSWrapper.cpp
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 class module for data receiving and transmitting in ROS
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Laboratory of Microdevices, Ltd. </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Laboratory of Microdevices nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "GKV_DeviceROSWrapper.h"

GKV_DeviceROSWrapper::GKV_DeviceROSWrapper(ros::NodeHandle *nh, std::string serial_port, uint32_t baudrate, uint8_t mode)
{
    //std::cout << serial_port <<std::endl;

    gkv_= new Gyrovert::GKV_Device(serial_port, baudrate);
    if (!(gkv_->GetSerialConnectionState())) GKV_Status = 1;
    received_adc_data_publisher=nh->advertise<gkv_ros_driver::GkvAdcData>("gkv_adc_data", 10);
    received_sensor_data_publisher=nh->advertise<gkv_ros_driver::GkvSensorsData>("gkv_sensors_data", 10);
    received_gyrovert_data_publisher=nh->advertise<gkv_ros_driver::GkvGyrovertData>("gkv_gyrovert_data", 10);
    received_inclinometer_data_publisher=nh->advertise<gkv_ros_driver::GkvInclinometerData>("gkv_inclinometer_data", 10);
    received_bins_data_publisher=nh->advertise<gkv_ros_driver::GkvBINSData>("gkv_bins_data", 10);
    received_gnss_data_publisher=nh->advertise<gkv_ros_driver::GkvGpsData>("gkv_gnss_data", 10);
    received_ext_gnss_data_publisher=nh->advertise<gkv_ros_driver::GkvExtGpsData>("gkv_ext_gnss_data", 10);
    received_custom_data_publisher=nh->advertise<gkv_ros_driver::GkvCustomData>("gkv_custom_data", 10);
    received_pose_stamped_publisher=nh->advertise<geometry_msgs::PoseStamped>("gkv_pose_stamped_data", 10);
    received_imu_data_publisher=nh->advertise<sensor_msgs::Imu>("gkv_imu_data", 10);
    received_nav_sat_fix_publisher=nh->advertise<sensor_msgs::NavSatFix>("gkv_nav_sat_fix_data", 10);

    ResetService = nh->advertiseService("gkv_reset_srv", &GKV_DeviceROSWrapper::ResetDevice,this);
    SetAlgorithmService = nh->advertiseService("gkv_set_alg_srv", &GKV_DeviceROSWrapper::SetAlgorithm,this);
    GetIDService = nh->advertiseService("gkv_get_id_srv", &GKV_DeviceROSWrapper::GetID,this);
    CheckConnectionService=nh->advertiseService("gkv_check_connection_srv", &GKV_DeviceROSWrapper::CheckConnection,this);
    SetPacketTypeService=nh->advertiseService("gkv_set_packet_type_srv", &GKV_DeviceROSWrapper::SetPacketType,this);
    SetCustomParametersService=nh->advertiseService("gkv_set_custom_params_srv", &GKV_DeviceROSWrapper::SetCustomParams,this);

    memset(&(device_id),0,sizeof(device_id));
    memcpy(&(device_id.serial_id),&NoDevStr,sizeof(NoDevStr));
    memcpy(&(device_id.description),&NoDevStr,sizeof(NoDevStr));

    if (!(GKV_Status))
    {
        gkv_->SetReceivedPacketCallback(std::bind(&GKV_DeviceROSWrapper::publishReceivedData, this, std::placeholders::_1));
        gkv_->RunDevice();
        MODE=mode;
        SetGKVAlgorithm(GKV_ESKF5_NAVIGATON_ALGORITHM);
        if (mode==GKV_ROS_PACKET_MODE)
        {
          if(SetGKVPacketType(GKV_SELECT_CUSTOM_PACKET))
          {
            SetGKVFabricCustomParams();
            SetGKVAccelerationUnits(GKV_MS2);
            SetGKVRateUnits(GKV_RADIANS_PER_SECOND);
            SetGKVAngleUnits(GKV_DEGREES);
          }
        }
        else
        {
          SetGKVPacketType(GKV_SELECT_DEFAULT_ALGORITHM_PACKET);
        }
    }
}

//RESET DEVICE FUNCTION
bool GKV_DeviceROSWrapper::ResetDevice(gkv_ros_driver::GkvReset::Request  &req,
                 gkv_ros_driver::GkvReset::Response &res)
{
    ResetRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (ResetRequestFlag==false)
        {
            break;
        }
        gkv_->ResetDevice();
        usleep(10000);
 //           ROS_INFO("Device Reset Req [%d]",i);
    }
    res.result=(!(ResetRequestFlag));
    return res.result;
}

//SET DEVICE ALGORITHM FUNCTION
bool GKV_DeviceROSWrapper::SetAlgorithm(gkv_ros_driver::GkvSetAlgorithm::Request  &req,
                 gkv_ros_driver::GkvSetAlgorithm::Response &res)
{
    if ((req.algorithm_number>9)||(req.algorithm_number==3))
    {
        return false;
    }
    SetAlgRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetAlgRequestFlag==false)
        {
            break;
        }
        gkv_->SetAlgorithm(req.algorithm_number);
        usleep(10000);
//            ROS_INFO("Device Set Alg Req [%d]",i);
    }
    res.result=(!(SetAlgRequestFlag));
    return res.result;
}

//SET DEVICE ALGORITHM FUNCTION
bool GKV_DeviceROSWrapper::SetGKVAlgorithm(uint8_t algorithm_number)
{
    if ((algorithm_number>9)||(algorithm_number==3))
    {
        return false;
    }
    SetAlgRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetAlgRequestFlag==false)
        {
            break;
        }
        gkv_->SetAlgorithm(algorithm_number);
        usleep(10000);
//            ROS_INFO("Device Set Alg Req [%d]",i);
    }
    return (!(SetAlgRequestFlag));
}

bool GKV_DeviceROSWrapper::SetGKVAccelerationUnits(uint8_t units)
{
    if(units>1)
    {
        return false;
    }
    SetAccelUnitsRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetAccelUnitsRequestFlag==false)
        {
            break;
        }
        gkv_->SetAccelerationUnits(units);
;
        usleep(10000);
//            ROS_INFO("Device Set Alg Req [%d]",i);
    }
    return (!(SetAccelUnitsRequestFlag));
}

bool GKV_DeviceROSWrapper::SetGKVRateUnits(uint8_t units)
{
    if(units>1)
    {
        return false;
    }
    SetRateUnitsRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetRateUnitsRequestFlag==false)
        {
            break;
        }
        gkv_->SetAngularRateUnits(units);
;
        usleep(10000);
//            ROS_INFO("Device Set Alg Req [%d]",i);
    }
    return (!(SetRateUnitsRequestFlag));
}

bool GKV_DeviceROSWrapper::SetGKVAngleUnits(uint8_t units)
{
    if(units>1)
    {
        return false;
    }
    SetAngleUnitsRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetAngleUnitsRequestFlag==false)
        {
            break;
        }
        gkv_->SetAngleUnits(units);
;
        usleep(10000);
//            ROS_INFO("Device Set Alg Req [%d]",i);
    }
    return (!(SetAngleUnitsRequestFlag));
}

//SET DEVICE PACKET TYPE FUNCTION
bool GKV_DeviceROSWrapper::SetPacketType(gkv_ros_driver::GkvSetPacketType::Request  &req,
                 gkv_ros_driver::GkvSetPacketType::Response &res)
{
    if ((!(req.packet_type==GKV_SELECT_DEFAULT_ALGORITHM_PACKET))&&(!(req.packet_type==GKV_SELECT_CUSTOM_PACKET)))
    {
        return false;
    }
    SetPacketTypeRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {

        if (SetPacketTypeRequestFlag==false)
        {
            break;
        }
        if (req.packet_type==0)
        {
          gkv_->SetDefaultAlgorithmPacket();
        }
        else {
          gkv_->SetCustomAlgorithmPacket();
        }
        usleep(10000);
//            ROS_INFO("Device Set Packet Type Req [%d]",i);
    }
    res.result=(!(SetPacketTypeRequestFlag));
    return res.result;
}

//SET DEVICE PACKET TYPE FUNCTION
bool GKV_DeviceROSWrapper::SetGKVPacketType(uint8_t packet_type)
{
    if ((!(packet_type==GKV_SELECT_DEFAULT_ALGORITHM_PACKET))&&(!(packet_type==GKV_SELECT_CUSTOM_PACKET)))
    {
        return false;
    }
    SetPacketTypeRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {

        if (SetPacketTypeRequestFlag==false)
        {
            break;
        }
        if (packet_type==GKV_SELECT_DEFAULT_ALGORITHM_PACKET)
        {
          gkv_->SetDefaultAlgorithmPacket();
        }
        else {
          gkv_->SetCustomAlgorithmPacket();
        }
        usleep(10000);
//            ROS_INFO("Device Set Packet Type Req [%d]",i);
    }
    return (!(SetPacketTypeRequestFlag));
}


//SET DEVICE CUSTOM PARAMS FUNCTION
bool GKV_DeviceROSWrapper::SetCustomParams(gkv_ros_driver::GkvSetCustomParameters::Request  &req,
                 gkv_ros_driver::GkvSetCustomParameters::Response &res)
{
    //check for maximum quantity of parameters
    if ((req.quantity_of_params>custom_params_limit))
    {
        return false;
    }
    //check for selected quantity of parameters is identical to array size
    if (!(req.quantity_of_params==req.params.size()))
    {
      return false;
    }
    //check for maximum number of custom gkv parameter
    for (uint8_t j=0;j<req.params.size();j++)
    {
        if (req.params[j]>max_number_of_custom_parameter) return false;
    }
    //prepare for request
    SetCustomParametersRequestFlag=true;
    //create buffer structure of parameter numbers packet
    Gyrovert::GKV_CustomDataParam required_custom_parameters;
    memset(&required_custom_parameters,0,sizeof(required_custom_parameters));
    //set buffer structure quantity
    required_custom_parameters.num=req.quantity_of_params;
    //set buffer structure numbers
    for (uint8_t k=0;k<req.quantity_of_params;k++)
    {
        required_custom_parameters.param[k]=req.params[k];
    }
    //send parameters packet
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetCustomParametersRequestFlag==false)
        {
            break;
        }
//        ROS_INFO("Device Custom Parameters Number [%d]",required_custom_parameters.num);
//        for (uint8_t m=0;m<req.quantity_of_params;m++)
//        {
//          ROS_INFO("Device Custom Parameter [%d]",required_custom_parameters.param[m]);
//        }
        gkv_->SetCustomPacketParam(&(required_custom_parameters.param[0]), required_custom_parameters.num);
        usleep(10000);
    }
    res.result=(!(SetCustomParametersRequestFlag));
    //if parameters written correctly set current device parameters to required
    if (res.result==true)
    {
        memcpy(&device_custom_parameters,&required_custom_parameters,sizeof(required_custom_parameters));
    }
    return res.result;
}


//SET DEVICE CUSTOM PARAMS FUNCTION
void GKV_DeviceROSWrapper::SetGKVFabricCustomParams()
{
    uint8_t params[]={GKV_STATUS,GKV_SAMPLE_COUNTER,
                      GKV_X,GKV_Y,GKV_Z,//position
                      GKV_Q1,GKV_Q2,GKV_Q3,GKV_Q0,//quaternion
                      GKV_WX,GKV_WY,GKV_WZ,//rate
                      GKV_AX,GKV_AY,GKV_AZ,//acceleration
                      GKV_ALG_VAR_PSI,GKV_ALG_VAR_THETA,GKV_ALG_VAR_PHI,//orientation covariance
                      GKV_ALG_VAR_X,GKV_ALG_VAR_Y,GKV_ALG_VAR_Z,//position covariance
                      GKV_ALG_INT_LAT,GKV_ALG_INT_LON,GKV_ALG_ALT,// gnss coordinates
                      GKV_GNSS_STATUS
                     };
    uint8_t quantity_of_params;
    quantity_of_params=sizeof(params);
    //check for maximum quantity of parameters
    //prepare for request
    SetCustomParametersRequestFlag=true;
    //create buffer structure of parameter numbers packet
    Gyrovert::GKV_CustomDataParam required_custom_parameters;
    memset(&required_custom_parameters,0,sizeof(required_custom_parameters));
    //set buffer structure quantity
    required_custom_parameters.num=quantity_of_params;
    //set buffer structure numbers
    for (uint8_t k=0;k<quantity_of_params;k++)
    {
        required_custom_parameters.param[k]=params[k];
    }
    //send parameters packet
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (SetCustomParametersRequestFlag==false)
        {
            break;
        }
//        ROS_INFO("Device Custom Parameters Number [%d]",required_custom_parameters.num);
//        for (uint8_t m=0;m<req.quantity_of_params;m++)
//        {
//          ROS_INFO("Device Custom Parameter [%d]",required_custom_parameters.param[m]);
//        }
        gkv_->SetCustomPacketParam(&(required_custom_parameters.param[0]), required_custom_parameters.num);
        usleep(10000);
    }
    bool result=(!(SetCustomParametersRequestFlag));
    //if parameters written correctly set current device parameters to required
    if (result==true)
    {
        memcpy(&device_custom_parameters,&required_custom_parameters,sizeof(required_custom_parameters));
    }
}


//GET DEVICE ID FUNCTION
bool GKV_DeviceROSWrapper::GetID(gkv_ros_driver::GkvGetID::Request  &req,
           gkv_ros_driver::GkvGetID::Response &res)
{
    RequestDevIDFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (RequestDevIDFlag==false)
        {
            break;
        }
        gkv_->RequestDeviceID();
        usleep(10000);
//            ROS_INFO("Device ID Req [%d]",i);
    }
    res.dev_description.data=device_id.description;
    res.dev_id.data=device_id.serial_id;
    if (RequestDevIDFlag==false)
    {
        return true;
    }
    else {
       RequestDevIDFlag=false;
       return false;
    }
}

//GET DEVICE ID FUNCTION
bool GKV_DeviceROSWrapper::CheckConnection(gkv_ros_driver::GkvCheckConnection::Request  &req,
           gkv_ros_driver::GkvCheckConnection::Response &res)
{
  if (!(IsConnected()))
  {
    return false;
  }
  else {
    CheckConnectionRequestFlag=true;
    for (uint8_t i=0;i<request_limit;i++)
    {
        if (CheckConnectionRequestFlag==false)
        {
            break;
        }
        gkv_->CheckConnection();
        usleep(10000);
//      ROS_INFO("Device Check Req [%d]",i);
    }
    res.result=(!(CheckConnectionRequestFlag));
    return res.result;
  }

}

void GKV_DeviceROSWrapper::publishReceivedData(Gyrovert::GKV_PacketBase * buf)
{
    char str[30];
    switch (buf->type)
    {
        case GKV_ADC_CODES_PACKET:
        {
            Gyrovert::GKV_ADCData* packet;
            packet = (Gyrovert::GKV_ADCData*)&buf->data;
            gkv_ros_driver::GkvAdcData msg;
            msg.acceleration_adc_x = packet->a[0];
            msg.acceleration_adc_y = packet->a[1];
            msg.acceleration_adc_z = packet->a[2];
            msg.angular_rate_adc_x = packet->w[0];
            msg.angular_rate_adc_y = packet->w[1];
            msg.angular_rate_adc_z = packet->w[2];
            received_adc_data_publisher.publish(msg);
            break;
        }
        case GKV_RAW_DATA_PACKET:
        {
            Gyrovert::GKV_RawData* packet;
            packet = (Gyrovert::GKV_RawData*)&buf->data;
            gkv_ros_driver::GkvSensorsData msg;
            msg.acceleration_x = packet->a[0];
            msg.acceleration_y = packet->a[1];
            msg.acceleration_z = packet->a[2];
            msg.angular_rate_x = packet->w[0];
            msg.angular_rate_y = packet->w[1];
            msg.angular_rate_z = packet->w[2];
            received_sensor_data_publisher.publish(msg);
            break;
        }
        case GKV_EULER_ANGLES_PACKET:
        {
            Gyrovert::GKV_GyrovertData* packet;
            packet = (Gyrovert::GKV_GyrovertData*)&buf->data;
            gkv_ros_driver::GkvGyrovertData msg;
            msg.yaw = packet->yaw;
            msg.pitch = packet->pitch;
            msg.roll = packet->roll;
            received_gyrovert_data_publisher.publish(msg);
            break;
        }
        case GKV_INCLINOMETER_PACKET:
        {
            Gyrovert::GKV_InclinometerData* packet;
            packet = (Gyrovert::GKV_InclinometerData*)&buf->data;
            gkv_ros_driver::GkvInclinometerData msg;
            msg.alpha = packet->alfa;
            msg.beta = packet->beta;
            received_inclinometer_data_publisher.publish(msg);
            break;
        }
        case GKV_BINS_PACKET:
        {
            Gyrovert::GKV_BINSData* packet;
            packet = (Gyrovert::GKV_BINSData*)&buf->data;
            gkv_ros_driver::GkvBINSData msg;
            msg.x = packet->x;
            msg.y = packet->y;
            msg.z = packet->z;
            msg.yaw = packet->yaw;
            msg.pitch = packet->pitch;
            msg.roll = packet->roll;
            msg.alpha = packet->alfa;
            msg.beta = packet->beta;
            msg.q0 = packet->q[0];
            msg.q1 = packet->q[1];
            msg.q2 = packet->q[2];
            msg.q3 = packet->q[3];
            received_bins_data_publisher.publish(msg);
            break;
        }
        case GKV_GNSS_PACKET:
        {
            Gyrovert::GKV_GpsData* packet;
            packet = (Gyrovert::GKV_GpsData*)&buf->data;
            gkv_ros_driver::GkvGpsData msg;
            msg.time = packet->time;
            msg.latitude = packet->latitude;
            msg.longitude = packet->longitude;
            msg.altitude = packet->altitude;
            msg.gps_state_status = packet->state_status;
            msg.TDOP = packet->TDOP;
            msg.HDOP = packet->HDOP;
            msg.VDOP = packet->VDOP;
            msg.horizontal_vel = packet->velocity;
            msg.azimuth = packet->yaw;
            msg.vertical_vel = packet->alt_velocity;
            received_gnss_data_publisher.publish(msg);
            break;
        }
        case GKV_EXTENDED_GNSS_PACKET:
        {
            Gyrovert::GKV_GpsDataExt* packet;
            packet = (Gyrovert::GKV_GpsDataExt*)&buf->data;
            gkv_ros_driver::GkvExtGpsData msg;
            msg.lat_vel = packet->vlat;
            msg.lon_vel = packet->vlon;
            msg.lat_std = packet->sig_lat;
            msg.lon_std = packet->sig_lon;
            msg.alt_std = packet->sig_alt;
            msg.lat_vel_std = packet->sig_vlat;
            msg.lon_vel_std = packet->sig_vlon;
            msg.alt_vel_std = packet->sig_valt;
            msg.num_ss = packet->num_ss;
//            ROS_INFO("Extended GNSS Data: lat_vel=[%f] lon_vel=[%f] lat_std=[%f] lon_std=[%f] alt_std=[%f] lat_vel_std=[%f] lon_vel_std=[%f] alt_vel_std=[%f] num_ss=[%d]",
//                            packet->vlat, packet->vlon, packet->sig_lat, packet->sig_lon, packet->sig_alt, packet->sig_vlat, packet->sig_vlon, packet->sig_valt, packet->num_ss);
            received_ext_gnss_data_publisher.publish(msg);
            break;
        }
        case GKV_CUSTOM_PACKET:
        {
            Gyrovert::GKV_CustomData* packet;
            packet = (Gyrovert::GKV_CustomData*)&buf->data;
            if (MODE==GKV_LMP_PACKET_MODE)
            {
              gkv_ros_driver::GkvCustomData msg;
              if (CustomParamNumbersReceived)
              {
                  msg.quantity_of_params=device_custom_parameters.num;
                  //select parameters that have uint32_t structure
                  for (uint8_t i=0;i<device_custom_parameters.num;i++)
                  {
                      msg.numbers.push_back(device_custom_parameters.param[i]);
                      if ((device_custom_parameters.param[i]==GKV_ALG_INT_LAT_NOPH)||
                          (device_custom_parameters.param[i]==GKV_ALG_INT_LON_NOPH)||
                          (device_custom_parameters.param[i]==GKV_ALG_INT_ALT_NOPH)||
                          (device_custom_parameters.param[i]==GKV_UTC_TIME)||
                          (device_custom_parameters.param[i]==GKV_GNSS_STATUS)||
                          (device_custom_parameters.param[i]==GKV_ALG_INT_LAT)||
                          (device_custom_parameters.param[i]==GKV_ALG_INT_LON)||
                          (device_custom_parameters.param[i]==GKV_GNSS_INT_LAT)||
                          (device_custom_parameters.param[i]==GKV_GNSS_INT_LON)||
                          (device_custom_parameters.param[i]==GKV_ALG_STATE_STATUS)||
                          (device_custom_parameters.param[i]==GKV_GPS_INT_X)||
                          (device_custom_parameters.param[i]==GKV_GPS_INT_Y)||
                          (device_custom_parameters.param[i]==GKV_GPS_INT_Z))
                      {
                          msg.param_values.push_back(*((int32_t *)&packet->parameter[i]));
                      }
                      else {
                          msg.param_values.push_back(packet->parameter[i]);
                      }
                  }
                //   std::cout << (*((int32_t *)&packet->parameter[35])) * 6.28 << "\n";
                //   std::cout << std::format("The answer is {}.\n", (*((int32_t *)&packet->parameter[35])) * 6.28);
                //   printf ("%d\n", (*((int32_t *)&packet->parameter[35])));
                  // printf ("original packet is %d\n", (*((int32_t *)&packet->parameter[35])));
                  // printf ("cast to float %f\n", _Float64(*((int32_t *)&packet->parameter[35])));

                //   printf ("msg is %d\n", msg.param_values[35]);
                  // printf("cast to int:\n");
                  // msg.param_values[35] = _Float64(*((int32_t *)&packet->parameter[35]));
                  // msg.param_values[36] = _Float64(*((int32_t *)&packet->parameter[36]));
                  // ROS_INFO_STREAM(msg.param_values[35]);
                  received_custom_data_publisher.publish(msg);
              }
              else
              {
                  gkv_->RequestCustomPacketParams();
              }
            }
            else {
              //------------------------------------------
              geometry_msgs::PoseStamped msg;
              //coordinates
              msg.pose.position.x=packet->parameter[2];
              msg.pose.position.y=packet->parameter[3];
              msg.pose.position.z=packet->parameter[4];
              //quaternion
              msg.pose.orientation.x=packet->parameter[5];
              msg.pose.orientation.x=packet->parameter[6];
              msg.pose.orientation.z=packet->parameter[7];
              msg.pose.orientation.w=packet->parameter[8];
              //----------------------------------------------
              sensor_msgs::Imu imu_msg;
              //quaternion
              imu_msg.orientation.x=packet->parameter[5];
              imu_msg.orientation.x=packet->parameter[6];
              imu_msg.orientation.z=packet->parameter[7];
              imu_msg.orientation.w=packet->parameter[8];
              //RATE
              imu_msg.angular_velocity.x=packet->parameter[9];
              imu_msg.angular_velocity.y=packet->parameter[10];
              imu_msg.angular_velocity.z=packet->parameter[11];
              //ACCELERATION
              imu_msg.linear_acceleration.x=packet->parameter[12];
              imu_msg.linear_acceleration.y=packet->parameter[13];
              imu_msg.linear_acceleration.z=packet->parameter[14];
              //orientation covariance
              imu_msg.orientation_covariance[0]=packet->parameter[15];
              imu_msg.orientation_covariance[1]=0;
              imu_msg.orientation_covariance[2]=0;
              imu_msg.orientation_covariance[3]=0;
              imu_msg.orientation_covariance[4]=packet->parameter[16];
              imu_msg.orientation_covariance[5]=0;
              imu_msg.orientation_covariance[6]=0;
              imu_msg.orientation_covariance[7]=0;
              imu_msg.orientation_covariance[8]=packet->parameter[17];
              //RATE covariance
              imu_msg.angular_velocity_covariance[0]=pow((0.059*pi/180/sqrt(20)),2);
              imu_msg.angular_velocity_covariance[1]=0;
              imu_msg.angular_velocity_covariance[2]=0;
              imu_msg.angular_velocity_covariance[3]=0;
              imu_msg.angular_velocity_covariance[4]=pow((0.059*pi/180/sqrt(20)),2);
              imu_msg.angular_velocity_covariance[5]=0;
              imu_msg.angular_velocity_covariance[6]=0;
              imu_msg.angular_velocity_covariance[7]=0;
              imu_msg.angular_velocity_covariance[8]=pow((0.059*pi/180/sqrt(20)),2);
              //Acceleration covariance
              imu_msg.linear_acceleration_covariance[0]=pow(((2.4/1000)*9.81/sqrt(20)),2);
              imu_msg.linear_acceleration_covariance[1]=0;
              imu_msg.linear_acceleration_covariance[2]=0;
              imu_msg.linear_acceleration_covariance[3]=0;
              imu_msg.linear_acceleration_covariance[4]=pow(((2.4/1000)*9.81/sqrt(20)),2);
              imu_msg.linear_acceleration_covariance[5]=0;
              imu_msg.linear_acceleration_covariance[6]=0;
              imu_msg.linear_acceleration_covariance[7]=0;
              imu_msg.linear_acceleration_covariance[8]=pow(((2.4/1000)*9.81/sqrt(20)),2);
              //----------------------------------------------
              sensor_msgs::NavSatFix nav_msg;
              //position
              nav_msg.latitude=(*((int32_t *)&(packet->parameter[21])))*(360/(pow(2,32)));
              nav_msg.longitude=(*((int32_t *)&(packet->parameter[22])))*(360/(pow(2,32)));
              nav_msg.altitude=packet->parameter[23];
              nav_msg.position_covariance[0]=packet->parameter[18];
              nav_msg.position_covariance[1]=0;
              nav_msg.position_covariance[2]=0;
              nav_msg.position_covariance[3]=0;
              nav_msg.position_covariance[4]=packet->parameter[19];
              nav_msg.position_covariance[5]=0;
              nav_msg.position_covariance[6]=0;
              nav_msg.position_covariance[7]=0;
              nav_msg.position_covariance[8]=packet->parameter[20];
              GNSS_SolutionFoundFlag=(((*((uint32_t *)&(packet->parameter[24])))>>10)&(3));//2d/3d solution found
              //GNSS_SolutionFoundFlag=(((*((uint32_t *)&(packet->parameter[24])))>>8));
              //GNSS_SolutionFoundFlag=(*((uint32_t *)&(packet->parameter[24])));
//              std::bitset<32> y((*((uint32_t *)&(packet->parameter[24]))));
//              std::cout << "gnss status " << y << '\n';
//              ROS_INFO("gnss status [%d]", GNSS_SolutionFoundFlag);
//              ROS_INFO("length [%d]",buf->preamble);

              if (GNSS_SolutionFoundFlag)
              {
                nav_msg.position_covariance_type=nav_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
                nav_msg.status.status=nav_msg.status.STATUS_FIX;
              }
              else {
                nav_msg.position_covariance_type=nav_msg.COVARIANCE_TYPE_UNKNOWN;
                nav_msg.status.status=nav_msg.status.STATUS_NO_FIX;
              }
              nav_msg.status.service=(nav_msg.status.SERVICE_COMPASS|nav_msg.status.SERVICE_GALILEO|nav_msg.status.SERVICE_GLONASS|nav_msg.status.SERVICE_GPS);//all services gps, glonass, galileo

              received_pose_stamped_publisher.publish(msg);
              received_imu_data_publisher.publish(imu_msg);
              received_nav_sat_fix_publisher.publish(nav_msg);

            }
            break;
        }
        case GKV_CUSTOM_DATA_PARAM_PACKET:
        {
            Gyrovert::GKV_CustomDataParam* packet;
            packet = (Gyrovert::GKV_CustomDataParam*)&buf->data;
            memcpy(&device_custom_parameters,packet,sizeof(Gyrovert::GKV_CustomDataParam));
            CustomParamNumbersReceived=true;
            break;
        }
        case GKV_CONFIRM_PACKET:
        {
            if (CheckConnectionRequestFlag)
            {
              CheckConnectionRequestFlag=false;
            }
            if (ResetRequestFlag)
            {
                ResetRequestFlag=false;
//                    ROS_INFO("Device Reseted");
            }
            if (SetAlgRequestFlag)
            {
                SetAlgRequestFlag=false;
//                    ROS_INFO("Algorithm changed");
            }
            if (SetPacketTypeRequestFlag)
            {
                SetPacketTypeRequestFlag=false;
//                    ROS_INFO("Packet Type changed");
            }
            if (SetCustomParametersRequestFlag)
            {
                SetCustomParametersRequestFlag=false;
//                    ROS_INFO("Custom parameters set");
            }
            if (SetAccelUnitsRequestFlag)
            {
              SetAccelUnitsRequestFlag=false;
            }
            if (SetAngleUnitsRequestFlag)
            {
              SetAngleUnitsRequestFlag=false;
            }
            if (SetRateUnitsRequestFlag)
            {
              SetRateUnitsRequestFlag=false;
            }
            break;
        }
        case GKV_DEV_ID_PACKET:
        {
            Gyrovert::GKV_ID* packet;
            packet = (Gyrovert::GKV_ID*)&buf->data;
            memcpy(&device_id,packet,sizeof(Gyrovert::GKV_ID));
            RequestDevIDFlag=false;
            break;
        }
    }
}
