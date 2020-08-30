/////////////////////////////////////////////////////////////////////////////
//
// GX4-25_Test.c
//
// Test program for the GX4-25
//
// Notes:  This program runs through most of the sdk functions supported
//         by the GX4-25.  It does not permanently alter any of the device
//         settings.
//
//
// External dependencies:
//
//  mip_sdk.h
//  GX4-25_Test.h
//
// Written By: Nathan Miller and Gregg Carpenter
//
//!@copyright 2014 Lord Microstrain Sensing Systems. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR 
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION 
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////
#include "GX5-25.h"
////////////////////////////////////////////////////////////////////////////////
//
//Using namespace 
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
////////////////////////////////////////////////////////////////////////////////

u8 enable_data_stats_output = 0;

//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count = 0;
u32 ahrs_valid_packet_count = 0;

u32 filter_timeout_packet_count = 0;
u32 ahrs_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count = 0;
u32 ahrs_checksum_error_packet_count = 0;

//Example data field storage

//AHRS
mip_ahrs_scaled_gyro  curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag   curr_ahrs_mag;
mip_ahrs_euler_angles curr_ahrs_euler_angles;
mip_ahrs_gps_timestamp curr_ahrs_gps_timestamp;
ahrs_scaled_pressure_mip_field curr_pressure;




//FILTER
mip_filter_status curr_filter_status;
mip_filter_timestamp curr_filter_timestamp;
mip_filter_attitude_euler_angles curr_filter_angles;
mip_filter_euler_attitude_uncertainty curr_filter_euler_attitude_uncertainty;
mip_filter_linear_acceleration curr_filter_linear_acceleration;
mip_filter_compensated_acceleration curr_filter_compensated_acceleration;
mip_filter_compensated_angular_rate curr_filter_compensated_angular_rate;
mip_filter_gyro_bias curr_filter_gyro_bias;
mip_filter_gyro_bias_uncertainty curr_filter_gyro_bias_uncertainty;
mip_filter_gravity_vector curr_filter_gravity_vector;
mip_filter_pressure_altitude_mip_field curr_filter_pressure_altitude_mip_field;



////////////////////////////////////////////////////////////////////////////////
//
// FILTER Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void filter_packet_callback(mip_EF_data *mip_EF_data_temp, u8 *packet, u16 packet_size, u8 callback_type)
{
	mip_field_header *field_header;
	u8               *field_data;
	u16              field_offset = 0;

	//The packet callback can have several types, process them all
	switch (callback_type)
	{
		///
		//Handle valid packets
		///
		case MIP_INTERFACE_CALLBACK_VALID_PACKET:
		{
			filter_valid_packet_count++;

			///
			//Loop through all of the data fields
			///
			while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
			{

				///
				// Decode the field
				///
				switch (field_header->descriptor)
				{
					///
					// Filter Status
					///
					case MIP_FILTER_DATA_FILTER_STATUS:
					{
						memcpy(&curr_filter_status, field_data, sizeof(mip_filter_status));
						mip_filter_status_byteswap(&curr_filter_status);
						mip_EF_data_temp->mip_filter_status = curr_filter_status;
					}

					///
					// Filter GPS Time
					///
					case MIP_FILTER_DATA_FILTER_TIMESTAMP:
					{
						memcpy(&curr_filter_timestamp, field_data, sizeof(mip_filter_timestamp));
						mip_filter_timestamp_byteswap(&curr_filter_timestamp);
						mip_EF_data_temp->mip_filter_timestamp = curr_filter_timestamp;
					}

					///
					// Estimated Attitude, Euler Angles
					///
					case MIP_FILTER_DATA_ATT_EULER_ANGLES:
					{
						memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));
						//For little-endian targets, byteswap the data field
						mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);
						mip_EF_data_temp->mip_filter_attitude_euler_angles = curr_filter_angles;
					}

					///
					// Estimated Attitude Uncertainty, Euler Angles
					///
					case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER:
					{
						memcpy(&curr_filter_euler_attitude_uncertainty, field_data, sizeof(mip_filter_euler_attitude_uncertainty));
						//For little-endian targets, byteswap the data field
						mip_filter_euler_attitude_uncertainty_byteswap(&curr_filter_euler_attitude_uncertainty);
						mip_EF_data_temp->mip_filter_euler_attitude_uncertainty = curr_filter_euler_attitude_uncertainty;
					}

					///
					// Estimated Acceleration, Linear
					///
					case MIP_FILTER_DATA_LINEAR_ACCELERATION:
					{
						memcpy(&curr_filter_linear_acceleration, field_data, sizeof(mip_filter_linear_acceleration));
						//For little-endian targets, byteswap the data field
						mip_filter_linear_acceleration_byteswap(&curr_filter_linear_acceleration);
						mip_EF_data_temp->mip_filter_linear_acceleration = curr_filter_linear_acceleration;
					}

					///
					// Estimated Acceleration, Compensated
					///
					case MIP_FILTER_DATA_COMPENSATED_ACCELERATION:
					{
						memcpy(&curr_filter_compensated_acceleration, field_data, sizeof(mip_filter_compensated_acceleration));
						//For little-endian targets, byteswap the data field
						mip_filter_compensated_acceleration_byteswap(&curr_filter_compensated_acceleration);
						mip_EF_data_temp->mip_filter_compensated_acceleration = curr_filter_compensated_acceleration;
					}

					///
					// Estimated Angular Rate, Compensated
					///
					case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE:
					{
						memcpy(&curr_filter_compensated_angular_rate, field_data, sizeof(mip_filter_compensated_angular_rate));
						//For little-endian targets, byteswap the data field
						mip_filter_compensated_angular_rate_byteswap(&curr_filter_compensated_angular_rate);
						mip_EF_data_temp->mip_filter_compensated_angular_rate = curr_filter_compensated_angular_rate;
					}

					///
					// Estimated Gyro Bias
					///
					case MIP_FILTER_DATA_GYRO_BIAS:
					{
						memcpy(&curr_filter_gyro_bias, field_data, sizeof(mip_filter_gyro_bias));
						//For little-endian targets, byteswap the data field
						mip_filter_gyro_bias_byteswap(&curr_filter_gyro_bias);
						mip_EF_data_temp->mip_filter_gyro_bias = curr_filter_gyro_bias;
					}

					///
					// Estimated Gyro Bias Uncertainty
					///
					case MIP_FILTER_DATA_GYRO_BIAS_UNCERTAINTY:
					{
						memcpy(&curr_filter_gyro_bias_uncertainty, field_data, sizeof(mip_filter_gyro_bias_uncertainty));
						//For little-endian targets, byteswap the data field
						mip_filter_gyro_bias_uncertainty_byteswap(&curr_filter_gyro_bias_uncertainty);
						mip_EF_data_temp->mip_filter_gyro_bias_uncertainty = curr_filter_gyro_bias_uncertainty;
					}

					///
					// Estimated Pressure Altitude
					///
					case MIP_FILTER_DATA_PRESSURE_ALTITUDE:
					{
						memcpy(&curr_filter_pressure_altitude_mip_field, field_data, sizeof(mip_filter_pressure_altitude_mip_field));
						mip_filter_pressure_altitude_byteswap(&curr_filter_pressure_altitude_mip_field);
						mip_EF_data_temp->mip_filter_pressure_altitude_mip_field = curr_filter_pressure_altitude_mip_field;
					}
					break;

					default: break;
				}
			}
		}break;


		///
		//Handle checksum error packets
		///
		case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
		{
			filter_checksum_error_packet_count++;
		}break;

		///
		//Handle timeout packets
		///
		case MIP_INTERFACE_CALLBACK_TIMEOUT:
		{
			filter_timeout_packet_count++;
		}break;
		default: break;
	}

	print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void ahrs_packet_callback(mip_ahrs_data *mip_ahrs_data_temp, u8 *packet, u16 packet_size, u8 callback_type)
{
	mip_field_header *field_header;
	u8               *field_data;
	u16              field_offset = 0;

	//The packet callback can have several types, process them all
	switch (callback_type)
	{
		///
		//Handle valid packets
		///

	case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	{
		ahrs_valid_packet_count++;

		///
		//Loop through all of the data fields
		///

		while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
		{

			///
			// Decode the field
			///

			switch (field_header->descriptor)
			{
				///
				// Scaled Accelerometer
				///

			case MIP_AHRS_DATA_ACCEL_SCALED:
			{
				memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);
				mip_ahrs_data_temp->mip_ahrs_scaled_accel = curr_ahrs_accel;

			}break;

			///
			// Scaled Gyro
			///

			case MIP_AHRS_DATA_GYRO_SCALED:
			{
				memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);
				mip_ahrs_data_temp->mip_ahrs_scaled_gyro = curr_ahrs_gyro;

			}break;

			///
			// Scaled Magnetometer
			///

			case MIP_AHRS_DATA_MAG_SCALED:
			{
				memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);
				mip_ahrs_data_temp->mip_ahrs_scaled_mag = curr_ahrs_mag;

			}break;

			case MIP_AHRS_DATA_EULER_ANGLES:
			{
				memcpy(&curr_ahrs_euler_angles, field_data, sizeof(mip_ahrs_euler_angles));

				//For little-endian targets, byteswap the data field
				mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler_angles);
				mip_ahrs_data_temp->mip_ahrs_euler_angles = curr_ahrs_euler_angles;
			}break;

			case MIP_AHRS_DATA_TIME_STAMP_GPS:
			{
				memcpy(&curr_ahrs_gps_timestamp, field_data, sizeof(mip_ahrs_gps_timestamp));

				//For little-endian targets, byteswap the data field
				mip_ahrs_gps_timestamp_byteswap(&curr_ahrs_gps_timestamp);
				mip_ahrs_data_temp->mip_ahrs_gps_timestamp = curr_ahrs_gps_timestamp;
			}break;

			default: break;
			}
		}
	}break;

	///
	//Handle checksum error packets
	///

	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
	{
		ahrs_checksum_error_packet_count++;
	}break;

	///
	//Handle timeout packets
	///

	case MIP_INTERFACE_CALLBACK_TIMEOUT:
	{
		ahrs_timeout_packet_count++;
	}break;
	default: break;
	}

	print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Command-Line Help
//
////////////////////////////////////////////////////////////////////////////////

void print_command_line_usage()
{
	printf("\n\n");
	printf("Usage:\n");
	printf("-----------------------------------------------------------------------\n\n");

	printf("   GX4-25_Test [com_port_num] [baudrate]\n");
	printf("\n\n");
	printf("   Example: \"GX4-25_Test 1 115200\", Opens a connection to the \n");
	printf("             GX4-25 on COM1, with a baudrate of 115200.\n");
	printf("\n\n");
	printf("   [ ] - required command input.\n");
	printf("\n-----------------------------------------------------------------------\n");
	printf("\n\n");
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Header
//
////////////////////////////////////////////////////////////////////////////////

void print_header()
{
	printf("\n");
	printf("GX4-25 Test Program\n");
	printf("Copyright 2014. LORD Microstrain Sensing Systems\n\n");
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats()
{
	if (enable_data_stats_output)
	{
		printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)   Packets", filter_valid_packet_count, filter_timeout_packet_count + filter_checksum_error_packet_count,
			ahrs_valid_packet_count, ahrs_timeout_packet_count + ahrs_checksum_error_packet_count);
	}
}


///////////////////////////////////////////////////////////////////////////////
//
// Device specific Status Acquisition Routines
//
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////// 
//                                                                              
//! @fn                                                                         
//! u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//                                                                              
//! @section DESCRIPTION                                                        
//! Requests GX4-IMU Basic or Diagnostic Status Message.                        
//                                                                              
//! @section DETAILS                                                            
//!                                                                             
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4 IMU (6237)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//                                                                              
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n        
//                                                                              
//! @section NOTES                                                              
//!                                                                             
//! This function should only be called in IMU Direct Mode.                     
/////////////////////////////////////////////////////////////////////////////// 

u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{
	gx4_imu_basic_status_field *basic_ptr;
	gx4_imu_diagnostic_device_status_field *diagnostic_ptr;
	u16 response_size = MIP_FIELD_HEADER_SIZE;

	if (status_selector == GX4_IMU_BASIC_STATUS_SEL)
		response_size += sizeof(gx4_imu_basic_status_field);
	else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
		response_size += sizeof(gx4_imu_diagnostic_device_status_field);

	while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK) {}

	if (status_selector == GX4_IMU_BASIC_STATUS_SEL)
	{
		if (response_size != sizeof(gx4_imu_basic_status_field))
			return MIP_INTERFACE_ERROR;
		else if (MIP_SDK_CONFIG_BYTESWAP)
		{
			basic_ptr = (gx4_imu_basic_status_field *)response_buffer;

			byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
			byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
			byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
		}
	}
	else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
	{

		if (response_size != sizeof(gx4_imu_diagnostic_device_status_field))
			return MIP_INTERFACE_ERROR;
		else if (MIP_SDK_CONFIG_BYTESWAP)
		{
			diagnostic_ptr = (gx4_imu_diagnostic_device_status_field *)response_buffer;

			byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
			byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
			byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
			byteswap_inplace(&diagnostic_ptr->gyro_range, sizeof(diagnostic_ptr->gyro_range));
			byteswap_inplace(&diagnostic_ptr->mag_range, sizeof(diagnostic_ptr->mag_range));
			byteswap_inplace(&diagnostic_ptr->pressure_range, sizeof(diagnostic_ptr->pressure_range));
			byteswap_inplace(&diagnostic_ptr->temp_degc, sizeof(diagnostic_ptr->temp_degc));
			byteswap_inplace(&diagnostic_ptr->last_temp_read_ms, sizeof(diagnostic_ptr->last_temp_read_ms));
			byteswap_inplace(&diagnostic_ptr->num_gps_pps_triggers, sizeof(diagnostic_ptr->num_gps_pps_triggers));
			byteswap_inplace(&diagnostic_ptr->last_gps_pps_trigger_ms, sizeof(diagnostic_ptr->last_gps_pps_trigger_ms));
			byteswap_inplace(&diagnostic_ptr->dropped_packets, sizeof(diagnostic_ptr->dropped_packets));
			byteswap_inplace(&diagnostic_ptr->com_port_bytes_written, sizeof(diagnostic_ptr->com_port_bytes_written));
			byteswap_inplace(&diagnostic_ptr->com_port_bytes_read, sizeof(diagnostic_ptr->com_port_bytes_read));
			byteswap_inplace(&diagnostic_ptr->com_port_write_overruns, sizeof(diagnostic_ptr->com_port_write_overruns));
			byteswap_inplace(&diagnostic_ptr->com_port_read_overruns, sizeof(diagnostic_ptr->com_port_read_overruns));
		}
	}
	else
		return MIP_INTERFACE_ERROR;

	return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////// 
//                                                                              
//! @fn                                                                         
//! u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//                                                                              
//! @section DESCRIPTION                                                        
//! Requests GX4-25 Basic or Diagnostic Status Message.                            
//                                                                              
//! @section DETAILS                                                            
//!                                                                             
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4-25 (6236)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//                                                                              
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n        
//                                                                              
//! @section NOTES                                                              
//!                                                                             
/////////////////////////////////////////////////////////////////////////////// 

u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{

	gx4_25_basic_status_field *basic_ptr;
	gx4_25_diagnostic_device_status_field *diagnostic_ptr;
	u16 response_size = MIP_FIELD_HEADER_SIZE;

	if (status_selector == GX4_25_BASIC_STATUS_SEL)
		response_size += sizeof(gx4_25_basic_status_field);
	else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
		response_size += sizeof(gx4_25_diagnostic_device_status_field);

	while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK) {}

	if (status_selector == GX4_25_BASIC_STATUS_SEL)
	{

		if (response_size != sizeof(gx4_25_basic_status_field))
			return MIP_INTERFACE_ERROR;
		else if (MIP_SDK_CONFIG_BYTESWAP)
		{
			basic_ptr = (gx4_25_basic_status_field *)response_buffer;

			byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
			byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
			byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
		}

	}
	else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
	{
		if (response_size != sizeof(gx4_25_diagnostic_device_status_field))
			return MIP_INTERFACE_ERROR;
		else if (MIP_SDK_CONFIG_BYTESWAP)
		{
			diagnostic_ptr = (gx4_25_diagnostic_device_status_field *)response_buffer;

			byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
			byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
			byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
			byteswap_inplace(&diagnostic_ptr->imu_dropped_packets, sizeof(diagnostic_ptr->imu_dropped_packets));
			byteswap_inplace(&diagnostic_ptr->filter_dropped_packets, sizeof(diagnostic_ptr->filter_dropped_packets));
			byteswap_inplace(&diagnostic_ptr->com1_port_bytes_written, sizeof(diagnostic_ptr->com1_port_bytes_written));
			byteswap_inplace(&diagnostic_ptr->com1_port_bytes_read, sizeof(diagnostic_ptr->com1_port_bytes_read));
			byteswap_inplace(&diagnostic_ptr->com1_port_write_overruns, sizeof(diagnostic_ptr->com1_port_write_overruns));
			byteswap_inplace(&diagnostic_ptr->com1_port_read_overruns, sizeof(diagnostic_ptr->com1_port_read_overruns));
			byteswap_inplace(&diagnostic_ptr->imu_parser_errors, sizeof(diagnostic_ptr->imu_parser_errors));
			byteswap_inplace(&diagnostic_ptr->imu_message_count, sizeof(diagnostic_ptr->imu_message_count));
			byteswap_inplace(&diagnostic_ptr->imu_last_message_ms, sizeof(diagnostic_ptr->imu_last_message_ms));
		}
	}
	else
		return MIP_INTERFACE_ERROR;

	return MIP_INTERFACE_OK;

}
