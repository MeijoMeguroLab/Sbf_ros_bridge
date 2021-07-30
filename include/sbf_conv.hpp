
#include "rtklib_msgs/RtklibNav.h"
#include "sensor_msgs/NavSatFix.h"
#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include "sbf_structs.hpp"

#define RAD2DEG(x) (x*(180/M_PI))

void ecef2llh(double ecef_pos[3], double llh_pos[3])
{
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt(1 - pow(semi_minor_axis / semi_major_axis, 2.0));
  double a2 = sqrt((ecef_pos[0] * ecef_pos[0]) + (ecef_pos[1] * ecef_pos[1]));
  double a3 = 54 * (semi_minor_axis * semi_minor_axis) * (ecef_pos[2] * ecef_pos[2]);
  double a4 = (a2 * a2) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]) - (a1 * a1) * (semi_major_axis * semi_major_axis - semi_minor_axis * semi_minor_axis);
  double a5 = ((a1 * a1) * (a1 * a1) * a3 * (a2 * a2)) / (a4 * a4 * a4);
  double a6 = pow((1 + a5 + sqrt(a5 * a5 + 2 * a5)), 1.0 / 3.0);
  double a7 = a3 / (3 * pow((a6 + 1 / a6 + 1), 2.0) * a4 * a4);
  double a8 = sqrt(1 + 2 * (a1 * a1) * (a1 * a1) * a7);
  double a9 = -(a7 * (a1 * a1) * a2) / (1 + a8) + sqrt((semi_major_axis * semi_major_axis / 2) * (1 + 1 / a8) - (a7 * (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2])) / (a8 * (1 + a8)) - a7 * (a2 * a2) / 2);
  double a10 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (ecef_pos[2] * ecef_pos[2]));
  double a11 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]));
  double a12 = ((semi_minor_axis * semi_minor_axis) * ecef_pos[2]) / (semi_major_axis * a11);
  llh_pos[0] = atan((ecef_pos[2] + (a1 * (semi_major_axis / semi_minor_axis)) * (a1 * (semi_major_axis / semi_minor_axis)) * a12) / a2);
  llh_pos[1] = 0;

  if (ecef_pos[0] >= 0)
  {
    llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0]));
  }
  else
  {
    if (ecef_pos[0] < 0 && ecef_pos[1] >= 0)
    {
      llh_pos[1] = M_PI + (atan(ecef_pos[1] / ecef_pos[0]));
    }
    else
    {
      llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0])) - M_PI;
    }
  }

  llh_pos[2] = a10 * (1 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * a11));
}

uint16_t CRC16 (const void *data, size_t length)
{
	uint32_t  i;
	uint16_t  crc = 0;

	const uint8_t *buff = (const uint8_t *) data;
	for (i=0; i<length; i++)
	{
		crc = (crc << 8) ^ CRC_TABLE[ (crc >> 8) ^ buff[i] ];
	}

	return crc;
}

bool isValid(const void *data)
{
	const BlockHeader_t * block = (const BlockHeader_t *) data;
	uint16_t crc;
	crc = CRC16( &(block->ID), block->Length-2*sizeof(uint16_t) );
	return (crc ==  block->CRC) ? true:false;
}


bool decodePVTCartesian(const void *data, rtklib_msgs::RtklibNav* pRtklib_nav)
{
	bool vaild = true;
	PVTCartesian pvtcartesian;
	memcpy(&pvtcartesian, data, sizeof(pvtcartesian));

	if(pvtcartesian.error != 0){
		ROS_INFO("PVTCartesian error :: %d", pvtcartesian.error);
		vaild = false;
		return vaild;
	}

	double ecef[3] = { pvtcartesian.x, pvtcartesian.y, pvtcartesian.z };
	double llh[3];
	ecef2llh(ecef, llh);

	pRtklib_nav->tow = pvtcartesian.tow;
  pRtklib_nav->ecef_pos.x = ecef[0];
  pRtklib_nav->ecef_pos.y = ecef[1];
  pRtklib_nav->ecef_pos.z = ecef[2];
  pRtklib_nav->ecef_vel.x = pvtcartesian.vx;
  pRtklib_nav->ecef_vel.y = pvtcartesian.vy;
  pRtklib_nav->ecef_vel.z = pvtcartesian.vz;
	pRtklib_nav->status.header = pRtklib_nav->header;
  pRtklib_nav->status.latitude = RAD2DEG(llh[0]);
  pRtklib_nav->status.longitude = RAD2DEG(llh[1]);
  pRtklib_nav->status.altitude = llh[2];
    //fix = 0 , other = -1
		if(4 == pvtcartesian.mode & 0x0F){
			pRtklib_nav->status.status.status = 0;
		} else {
			pRtklib_nav->status.status.status = -1;
		}
  pRtklib_nav->status.status.service = 1;
  pRtklib_nav->status.position_covariance_type = 3;

	return vaild;
}

bool decodeReceiverStatus(const void *data)
{
  bool vaild = true;
  ReceiverStatus receiverstatus;
  memcpy(&receiverstatus, data, sizeof(receiverstatus));

  ROS_INFO("CPU_LOAD:: %d", receiverstatus.data.cpuload);

  return vaild;
}
