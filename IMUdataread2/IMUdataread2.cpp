// IMUdataread2.cpp : 定义控制台应用程序的入口点。
//

#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include "GX5-25.h"
#include "STIM300.h"

using namespace std;

const double GX4_G = 9.80665;/*% 3DM_GX4_25 G*/

int praseimudata(string filepath)
{

	FILE *fp = NULL;
	if ((fp = fopen(filepath.c_str(), "rb")) == NULL)
	{
		printf("Open imu datfile error!\n");
		return 0;
	}
	vector<mip_ahrs_data> GX4_data;
	vector<mip_EF_data> GX4_EF_data;
	vector<STIM300_packet> STIM300_data;
	u8 STIM_input_buffer[STIM_DATA_SIZE];
	u8 GX4_input_buffer[MIP_INTERFACE_INPUT_RING_BUFFER_SIZE];
	u8 byte_temp1;
	u8 byte_temp2;
	while (!feof(fp))
	{
		byte_temp1 = fgetc(fp);

		///
		//Valid STIM300 Packet Found
		///
		if (byte_temp1 == STIM_IDEN_BYTE1)
		{
			STIM_input_buffer[0] = byte_temp1;
			fread(&(STIM_input_buffer[1]), 1, STIM_DATA_SIZE - 1, fp);
			if (STIM_is_checksum_valid(STIM_input_buffer, STIM_INIDATA_SIZE))
			{
				STIM300_packet STIM300_packet_temp;
				STIM300_packet_callback(&STIM300_packet_temp, STIM_input_buffer, STIM_DATA_SIZE);
				STIM300_data.push_back(STIM300_packet_temp);
			}
			continue;
		}

		///
		//Valid 3DM Packet Found
		///
		if (byte_temp1 == MIP_SYNC_BYTE1)
		{
			GX4_input_buffer[0] = byte_temp1;
			byte_temp2 = fgetc(fp);
			if (byte_temp2 == MIP_SYNC_BYTE2)
			{
				GX4_input_buffer[1] = byte_temp2;

				u8 byte_temp = fgetc(fp);
				if (byte_temp == 0x80) // get the IMU data
				{
					GX4_input_buffer[2] = 0x80;
					int packet_length = fgetc(fp);
					GX4_input_buffer[3] = packet_length;
					fread(&(GX4_input_buffer[4]), 1, packet_length + 2, fp);
					packet_length = MIP_HEADER_SIZE + packet_length + MIP_CHECKSUM_SIZE;
					if (mip_is_checksum_valid(GX4_input_buffer) == MIP_OK)
					{
						//Trigger the callback with the valid packet
						mip_ahrs_data mip_ahrs_data_temp;
						ahrs_packet_callback(&mip_ahrs_data_temp, GX4_input_buffer, packet_length, MIP_INTERFACE_CALLBACK_VALID_PACKET);
						GX4_data.push_back(mip_ahrs_data_temp);
					}
				}
				else if (byte_temp == 0x82) // get the EF data
				{
					GX4_input_buffer[2] = 0x82;
					int packet_length = fgetc(fp);
					GX4_input_buffer[3] = packet_length;
					fread(&(GX4_input_buffer[4]), 1, packet_length + 2, fp);
					packet_length = MIP_HEADER_SIZE + packet_length + MIP_CHECKSUM_SIZE;
					if (mip_is_checksum_valid(GX4_input_buffer) == MIP_OK)
					{
						//Trigger the callback with the valid packet
						mip_EF_data mip_EF_data_temp;
						filter_packet_callback(&mip_EF_data_temp, GX4_input_buffer, packet_length, MIP_INTERFACE_CALLBACK_VALID_PACKET);
						GX4_EF_data.push_back(mip_EF_data_temp);
					}
				}
			}
			continue;
		}
	}

	///
	//Valid 3DM Packet Found
	///
	if (GX4_data.size() != 0)
	{
		string::size_type dotpos = filepath.rfind(".");
		string savefilepath = filepath.substr(0, dotpos) + "_AHRS.dat";
		std::ofstream save_imu;
		save_imu.open(savefilepath);
		for (vector<mip_ahrs_data>::iterator it = GX4_data.begin(); it != GX4_data.end(); it++)
		{
			save_imu << std::setiosflags(std::ios::fixed) << std::setprecision(3) << it->mip_ahrs_gps_timestamp.tow << ' '
				<< std::setiosflags(std::ios::fixed) << std::setprecision(9)
				<< it->mip_ahrs_scaled_accel.scaled_accel[0] * GX4_G << ' '//accb:m/s^2
				<< it->mip_ahrs_scaled_accel.scaled_accel[1] * GX4_G << ' '
				<< it->mip_ahrs_scaled_accel.scaled_accel[2] * GX4_G << ' '
				<< it->mip_ahrs_scaled_gyro.scaled_gyro[0] << ' ' //gyro:rad/s
				<< it->mip_ahrs_scaled_gyro.scaled_gyro[1] << ' '
				<< it->mip_ahrs_scaled_gyro.scaled_gyro[2] << ' '
				<< it->mip_ahrs_scaled_mag.scaled_mag[0] << ' '
				<< it->mip_ahrs_scaled_mag.scaled_mag[1] << ' '
				<< it->mip_ahrs_scaled_mag.scaled_mag[2] << ' '
				<< it->mip_ahrs_euler_angles.roll << ' ' //euler angle:rad
				<< it->mip_ahrs_euler_angles.pitch << ' '
				<< it->mip_ahrs_euler_angles.yaw << ' '
				<< std::endl;
		}
	}

	///
	//Valid 3DM Estimation Filter Packet Found
	///
	if (GX4_EF_data.size() != 0)
	{
		string::size_type dotpos = filepath.rfind(".");
		string saveEFfilepath1 = filepath.substr(0, dotpos) + "_EF.csv";
		std::ofstream save_imu_Estimation_Filter;
		save_imu_Estimation_Filter.open(saveEFfilepath1);
		save_imu_Estimation_Filter << "filter_state" << ',' << "status_flags" << ','
			<< "valid_flags" << ',' << "timestamp" << ','
			<< "valid_flags" << ',' << "roll" << ',' << "pitch" << ',' << "yaw" << ','
			<< "valid_flags" << ',' << "roll_uncertainty" << ',' << "pitch_uncertainty" << ',' << "yaw_uncertainty" << ','
			<< "valid_flags" << ',' << "linear_acc_x" << ',' << "linear_acc_y" << ',' << "linear_acc_z" << ','
			<< "valid_flags" << ',' << "compensated_acc_x" << ',' << "compensated_acc_y" << ',' << "compensated_acc_z" << ','
			<< "valid_flags" << ',' << "compensated_gyr_x" << ',' << "compensated_gyr_y" << ',' << "compensated_gyr_z" << ','
			<< "valid_flags" << ',' << "gyr_bias_x" << ',' << "gyr_bias_y" << ',' << "gyr_bias_z" << ','
			<< "valid_flags" << ',' << "gyr_bias_uncertainty_x" << ',' << "gyr_bias_uncertainty_y" << ',' << "gyr_bias_uncertainty_z" << ','
			<< "valid_flags" << ',' << "pressure_altitude" << std::endl;

		string saveEFfilepath2 = filepath.substr(0, dotpos) + "_Test.dat";
		std::ofstream save_imu_BaseData;
		save_imu_BaseData.open(saveEFfilepath2);

		double current_time = GX4_EF_data.begin()->mip_filter_timestamp.tow;
		for (vector<mip_EF_data>::iterator it = GX4_EF_data.begin(); it != GX4_EF_data.end(); it++)
		{
			if (it->mip_filter_timestamp.tow - current_time > 1.0)
				continue;
			else
				current_time = it->mip_filter_timestamp.tow;

			save_imu_Estimation_Filter << std::setiosflags(std::ios::fixed) << std::setprecision(9)
				<< it->mip_filter_status.filter_state << ',' << it->mip_filter_status.status_flags << ','
				<< it->mip_filter_timestamp.valid_flags << ',' << it->mip_filter_timestamp.tow << ','
				<< it->mip_filter_attitude_euler_angles.valid_flags << ',' //euler angle:rad
				<< it->mip_filter_attitude_euler_angles.roll << ','
				<< it->mip_filter_attitude_euler_angles.pitch << ','
				<< it->mip_filter_attitude_euler_angles.yaw << ','
				<< it->mip_filter_euler_attitude_uncertainty.valid_flags << ','//euler angle uncertainty:rad
				<< it->mip_filter_euler_attitude_uncertainty.roll << ','
				<< it->mip_filter_euler_attitude_uncertainty.pitch << ','
				<< it->mip_filter_euler_attitude_uncertainty.yaw << ','
				<< it->mip_filter_linear_acceleration.valid_flags << ',' //linear_accb:m/s^2
				<< it->mip_filter_linear_acceleration.x << ','
				<< it->mip_filter_linear_acceleration.y << ','
				<< it->mip_filter_linear_acceleration.z << ','
				<< it->mip_filter_compensated_acceleration.valid_flags << ','//compensated_accb:m/s^2
				<< it->mip_filter_compensated_acceleration.x << ','
				<< it->mip_filter_compensated_acceleration.y << ','
				<< it->mip_filter_compensated_acceleration.z << ','
				<< it->mip_filter_compensated_angular_rate.valid_flags << ','//compensated_gyro:rad/s
				<< it->mip_filter_compensated_angular_rate.x << ','
				<< it->mip_filter_compensated_angular_rate.y << ','
				<< it->mip_filter_compensated_angular_rate.z << ','
				<< it->mip_filter_gyro_bias.valid_flags << ','//Gyro bias:rad/s
				<< it->mip_filter_gyro_bias.x << ','
				<< it->mip_filter_gyro_bias.y << ','
				<< it->mip_filter_gyro_bias.z << ','
				<< it->mip_filter_gyro_bias_uncertainty.valid_flags << ','//Gyro bias uncertainty:rad/s
				<< it->mip_filter_gyro_bias_uncertainty.x << ','
				<< it->mip_filter_gyro_bias_uncertainty.y << ','
				<< it->mip_filter_gyro_bias_uncertainty.z << ','
				<< it->mip_filter_pressure_altitude_mip_field.valid_flags << ','//Pressure Altitude:m
				<< it->mip_filter_pressure_altitude_mip_field.pressure_altitude
				<< std::endl;

			save_imu_BaseData << std::setiosflags(std::ios::fixed) << std::setprecision(3) << it->mip_filter_timestamp.tow << ' '
				<< std::setprecision(9)
				<< it->mip_filter_compensated_acceleration.x << ' '//accb:m/s^2
				<< it->mip_filter_compensated_acceleration.y << ' '
				<< it->mip_filter_compensated_acceleration.z << ' '
				<< it->mip_filter_compensated_angular_rate.x << ' ' //gyro:rad/s(without bias)
				<< it->mip_filter_compensated_angular_rate.y << ' '
				<< it->mip_filter_compensated_angular_rate.z << ' '
				<< it->mip_filter_gyro_bias.x << ' ' //gyro bias:rad/s
				<< it->mip_filter_gyro_bias.y << ' '
				<< it->mip_filter_gyro_bias.z << ' '
				<< it->mip_filter_attitude_euler_angles.roll << ' ' //euler angle:rad
				<< it->mip_filter_attitude_euler_angles.pitch << ' '
				<< it->mip_filter_attitude_euler_angles.yaw
				<< std::endl;
		}

		save_imu_Estimation_Filter.close();
		save_imu_BaseData.close();
	}

	return 1;
}

int main(int argc, char* argv[])
{
	//std::string srcfilepath = "E:/VSprogram/data/201905241100_lecture_big/Long_route/ReceivedTofile-COM4-2019_5_24_11-39-07.DAT";

	string srcfilepath = argv[1];
	
	if (!praseimudata(srcfilepath))
	{
		std::cout << "IMUData input error" << std::endl;
		return 1;
	}
	else
	{
		cout << "IMUData Convert Succeed!!!" << endl;
	}
	return 0;
}

