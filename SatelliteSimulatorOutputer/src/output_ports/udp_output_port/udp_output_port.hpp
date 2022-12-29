#pragma once

#include "satellite_math.hpp"
#include "../output_port.hpp"
#include <string>

#include <iostream>
#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib") 
#pragma warning(disable:4996) 

#define SERVER "127.0.0.1"  // or "localhost" - ip address of UDP server
#define BUFLEN 512  // max length of answer
#define PORT 8888  // the port on which to listen for incoming data

namespace satellite_simulator_data_outputer
{

	class UdpOutputPort : public OutputPort
	{
	public:
		UdpOutputPort(const std::string &url, int port);
		~UdpOutputPort();

		virtual void output(const sat_math::Matrix& position_ECI,
			const sat_math::Matrix& attitude_ECI,
			const sat_math::Matrix& velocity_ECI,
			const sat_math::Matrix& angular_velocity,
			const sat_math::Matrix& magnet_field_ECI,
			const sat_math::Matrix& magnet_field_body,
			const sat_math::Matrix& sun_dir_ECI,
			const sat_math::Matrix& sun_dir_body);

	private:
		int _udp_socket;
		sockaddr_in _server_address;

		int prepare_package(const sat_math::Matrix& position_ECI,
			const sat_math::Matrix& attitude_ECI,
			const sat_math::Matrix& velocity_ECI,
			const sat_math::Matrix& angular_velocity,
			const sat_math::Matrix& magnet_field_ECI,
			const sat_math::Matrix& magnet_field_body,
			const sat_math::Matrix& sun_dir_ECI,
			const sat_math::Matrix& sun_dir_body,
			char* package);
	};

}
