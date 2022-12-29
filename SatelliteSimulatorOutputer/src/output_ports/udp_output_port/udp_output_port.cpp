#include "udp_output_port.hpp"

#include <iostream>

namespace satellite_simulator_data_outputer
{
	UdpOutputPort::UdpOutputPort(const std::string& url, int port)
	{

        // initialise winsock
        WSADATA ws;
        printf("Initialising Winsock...");
        if (WSAStartup(MAKEWORD(2, 2), &ws) != 0)
        {
            printf("Failed. Error Code: %d", WSAGetLastError());
            return;
        }
        printf("Initialised.\n");

        // create socket
        
        if ((_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) // <<< UDP socket
        {
            printf("socket() failed with error code: %d", WSAGetLastError());
            return;
        }

        // setup address structure
        memset((char*)&_server_address, 0, sizeof(_server_address));
        _server_address.sin_family = AF_INET;
        _server_address.sin_port = htons(port);
        _server_address.sin_addr.S_un.S_addr = inet_addr(url.c_str());

	}

    UdpOutputPort::~UdpOutputPort()
    {
        closesocket(_udp_socket);
        WSACleanup();
    }

	void UdpOutputPort::output(
		const sat_math::Matrix& position_ECI,
		const sat_math::Matrix& attitude_ECI,
		const sat_math::Matrix& velocity_ECI,
		const sat_math::Matrix& angular_velocity,
		const sat_math::Matrix& magnet_field_ECI,
		const sat_math::Matrix& magnet_field_body,
		const sat_math::Matrix& sun_dir_ECI,
		const sat_math::Matrix& sun_dir_body
	)
	{
        char package[128];
        int size_of_package = prepare_package(
            position_ECI,
            attitude_ECI,
            velocity_ECI,
            angular_velocity,
            magnet_field_ECI,
            magnet_field_body,
            sun_dir_ECI,
            sun_dir_body, 
            package
        );
        

        // send the message
        sendto(_udp_socket, package, size_of_package, 0, (sockaddr*)&_server_address, sizeof(sockaddr_in));
	}

    int UdpOutputPort::prepare_package(
        const sat_math::Matrix& position_ECI,
        const sat_math::Matrix& attitude_ECI,
        const sat_math::Matrix& velocity_ECI,
        const sat_math::Matrix& angular_velocity,
        const sat_math::Matrix& magnet_field_ECI,
        const sat_math::Matrix& magnet_field_body,
        const sat_math::Matrix& sun_dir_ECI,
        const sat_math::Matrix& sun_dir_body,
        char* package
    )
    {
        int offset = 0;
        memcpy(package + offset, &position_ECI(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &position_ECI(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &position_ECI(2, 0), 8);
        offset += 8;

        memcpy(package + offset, &attitude_ECI(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &attitude_ECI(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &attitude_ECI(2, 0), 8);
        offset += 8;
        memcpy(package + offset, &attitude_ECI(3, 0), 8);
        offset += 8;

        return offset;
    }
}