/*
 * sbf_udp.cpp
 *
 * Author Kashimoto
 * Ver 1.00 2021/04/22
 */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "rtklib_msgs/RtklibNav.h"
#include "sensor_msgs/NavSatFix.h"

#include "sbf_conv.hpp"

#define BUFFER_SAFE 2000
#define BUFFER_SIZE 2048

struct sockaddr_in dstAddr;
static ros::Publisher pub;
static ros::Publisher pub2;
static bool isConnected = false;

static void packet_receive_rate(int fd, std::string frame_id, double rate)
{
  rtklib_msgs::RtklibNav rtklib_nav;
  sensor_msgs::NavSatFix fix;
  rtklib_nav.header.frame_id = rtklib_nav.status.header.frame_id = fix.header.frame_id = frame_id.c_str();
  ros::Rate loop_rate(rate);

  int rem;
  int ret;
  bool chksum;
  char buffer[BUFFER_SIZE];
  char* w_buffer = buffer;
  char* r_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];
  bool header_enable = false;

  while (ros::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      pub.shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        ros::shutdown();
      }
    }

    ros::Time now = ros::Time::now();
    int buffer_num = (w_buffer - 1) - r_buffer;
    while((w_buffer - 1) - r_buffer > 8)
    {
      char* sync1 = strchr(r_buffer, '$');
      if ( (sync1 != NULL)&&( *(sync1+1) == '@' ) )
      {
        const BlockHeader_t * block = (const BlockHeader_t *) sync1;
        uint16_t id = block->ID & 0x01FFF;
        r_buffer = sync1;
        r_buffer += block->Length;
        ROS_INFO("ID == %d, block->Length = %d", id,block->Length);
        header_enable = true;
        if(isValid(sync1))
        {
          header_enable = false;
          if( id == 4006 ){
            /* PVTCartesian */
            rtklib_nav.header.stamp = rtklib_nav.status.header.stamp = fix.header.stamp = now;
            decodePVTCartesian(sync1, &rtklib_nav);

            pub.publish(rtklib_nav);

            fix.latitude = rtklib_nav.status.latitude;
            fix.longitude = rtklib_nav.status.longitude;
            fix.altitude = rtklib_nav.status.altitude;
            fix.status.status = rtklib_nav.status.status.status;
            fix.status.service = rtklib_nav.status.status.service;
            pub2.publish(fix);

          } else if( id == 4014 ) {
            /* Receiverstatus */
            decodeReceiverStatus(sync1);
          } else {
            /* undefine message */

          }
        }
      }else{
        break;
      }
    }

    rem = w_buffer - r_buffer;

    if (rem > BUFFER_SAFE)
    {
      ROS_WARN("Buffer over.Init Buffer.");
      rem = 0;
      ret = 0;
      w_buffer = buffer;
      r_buffer = buffer;
      memset(buffer, 0x00, BUFFER_SIZE);
    }else if(header_enable == true){
      ROS_INFO("wait data part.");
      memmove(buffer, r_buffer, rem);
      w_buffer = buffer + rem;
    }else if(rem == 0){
      ret = 0;
      w_buffer = buffer;
      r_buffer = buffer;
      memset(buffer, 0x00, BUFFER_SIZE);
    }else{

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  close(fd);
}

static void packet_receive_no_rate(int fd, std::string frame_id)
{
  rtklib_msgs::RtklibNav rtklib_nav;
  sensor_msgs::NavSatFix fix;
  rtklib_nav.header.frame_id = rtklib_nav.status.header.frame_id = fix.header.frame_id = frame_id.c_str();

  int rem;
  int ret;
  bool chksum;
  char buffer[BUFFER_SIZE];
  char* w_buffer = buffer;
  char* r_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];
  bool header_enable = false;

  while (ros::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      pub.shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        ros::shutdown();
      }
    }

    ros::Time now = ros::Time::now();
    int buffer_num = (w_buffer - 1) - r_buffer;
    while((w_buffer - 1) - r_buffer > 8)
    {
      char* sync1 = strchr(r_buffer, '$');
      if ( (sync1 != NULL)&&( *(sync1+1) == '@' ) )
      {
        const BlockHeader_t * block = (const BlockHeader_t *) sync1;
        uint16_t id = block->ID & 0x01FFF;
        r_buffer = sync1;
        r_buffer += block->Length;
        ROS_INFO("ID == %d, block->Length = %d", id,block->Length);
        header_enable = true;
        if(isValid(sync1))
        {
          header_enable = false;
          if( id == 4006 ){
            /* PVTCartesian */
            rtklib_nav.header.stamp = rtklib_nav.status.header.stamp = fix.header.stamp = now;
            decodePVTCartesian(sync1, &rtklib_nav);

            pub.publish(rtklib_nav);

            fix.latitude = rtklib_nav.status.latitude;
            fix.longitude = rtklib_nav.status.longitude;
            fix.altitude = rtklib_nav.status.altitude;
            fix.status.status = rtklib_nav.status.status.status;
            fix.status.service = rtklib_nav.status.status.service;
            pub2.publish(fix);

          } else if( id == 4014 ) {
            /* Receiverstatus */
            decodeReceiverStatus(sync1);
          } else {
            /* undefine message */

          }
        }
      }else{
        break;
      }
    }

    rem = w_buffer - r_buffer;

    if (rem > BUFFER_SAFE)
    {
      ROS_WARN("Buffer over.Init Buffer.");
      rem = 0;
      ret = 0;
      w_buffer = buffer;
      r_buffer = buffer;
      memset(buffer, 0x00, BUFFER_SIZE);
    }else if(header_enable == true){
      ROS_INFO("wait data part.");
      memmove(buffer, r_buffer, rem);
      w_buffer = buffer + rem;
    }else if(rem == 0){
      ret = 0;
      w_buffer = buffer;
      r_buffer = buffer;
      memset(buffer, 0x00, BUFFER_SIZE);
    }else{

    }

    ros::spinOnce();
  }
  close(fd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sbf_udp");
  ros::NodeHandle node_handle_;

  int port,sock;
  std::string address, sbf_topic, frame_id;
  int result,val;
  double rate;

  // Read parameters
  node_handle_.param("/sbf_udp/address", address, std::string("127.0.0.1"));
  node_handle_.param("/sbf_udp/port", port, 28004);
  node_handle_.param("/sbf_udp/sbf_topic", sbf_topic, std::string("rtklib_nav"));
  node_handle_.param("/sbf_udp/frame_id", frame_id, std::string("gps"));
  node_handle_.param("/sbf_udp/rate", rate, 0.0);

  pub = node_handle_.advertise<rtklib_msgs::RtklibNav>(sbf_topic, 10);

  ROS_INFO("IP: %s", address.c_str());
  ROS_INFO("PORT: %d", port);
  ROS_INFO("RATE: %.1lf", rate);

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_family = AF_INET;
  dstAddr.sin_addr.s_addr = INADDR_ANY;
  dstAddr.sin_port = htons(port);

  ros::Rate loop_rate(1.0);
  while (ros::ok())
  {
    result = bind(sock, (struct sockaddr *)&dstAddr, sizeof(dstAddr));
    if( result < 0 )
    {
      /* non-connect */
      ROS_INFO("CONNECT TRY...");
    }
    else
    {
      /* connect */
      isConnected = true;
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  if ((isConnected == true)&&(ros::ok()))
  {
    if( rate != 0.0 )
    {
      /* non-block *//* ros::rate() */
      val = 1;
      ioctl(sock, FIONBIO, &val);
      packet_receive_rate(sock, frame_id, rate);
    }
    else
    {
      /* block */
      packet_receive_no_rate(sock, frame_id);
    }

  }

  return 0;
}
