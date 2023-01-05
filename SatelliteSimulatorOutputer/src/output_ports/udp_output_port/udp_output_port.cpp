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
		const sat_math::Matrix& sun_dir_body,
		const sat_math::Matrix& magnet_field_body
	)
	{
        char package[512];
        int size_of_package = prepare_package(
            position_ECI,
            attitude_ECI,
            velocity_ECI,
            angular_velocity,
            sun_dir_body, 
            magnet_field_body,
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
        const sat_math::Matrix& sun_dir_body,
        const sat_math::Matrix& magnet_field_body,
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

        memcpy(package + offset, &velocity_ECI(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &velocity_ECI(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &velocity_ECI(2, 0), 8);
        offset += 8;

        memcpy(package + offset, &angular_velocity(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &angular_velocity(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &angular_velocity(2, 0), 8);
        offset += 8;

        memcpy(package + offset, &sun_dir_body(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &sun_dir_body(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &sun_dir_body(2, 0), 8);
        offset += 8;

        memcpy(package + offset, &magnet_field_body(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &magnet_field_body(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &magnet_field_body(2, 0), 8);
        offset += 8;

        sat_math::Matrix euler = sat_math::QuaternionOperations::quaternion_to_euler_angles_ZYX(attitude_ECI);
        memcpy(package + offset, &euler(0, 0), 8);
        offset += 8;
        memcpy(package + offset, &euler(1, 0), 8);
        offset += 8;
        memcpy(package + offset, &euler(2, 0), 8);
        offset += 8;

        return offset;
    }
}