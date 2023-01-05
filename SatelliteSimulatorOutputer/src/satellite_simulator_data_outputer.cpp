#include "satellite_simulator_data_outputer.hpp"
#include "output_ports/file_output_port/file_output_port.hpp"
#include "output_ports/udp_output_port/udp_output_port.hpp"


namespace satellite_simulator_data_outputer
{

	SatelliteSimulatorDataOutputer::SatelliteSimulatorDataOutputer(const std::string& output_directory, const std::string& ip_address, int port)
	{
		_output_ports.emplace_back(new FileOutputPort(output_directory + '/'));
		_output_ports.emplace_back(new UdpOutputPort(ip_address, port));
	}

	SatelliteSimulatorDataOutputer::~SatelliteSimulatorDataOutputer()
	{

	}

	void SatelliteSimulatorDataOutputer::output(
		const sat_math::Matrix& position_ECI,
		const sat_math::Matrix& attitude_ECI,
		const sat_math::Matrix& velocity_ECI,
		const sat_math::Matrix& angular_velocity,
		const sat_math::Matrix& sun_dir_body,
		const sat_math::Matrix& magnet_field_body
	)
	{
		for (int i = 0; i < _output_ports.size(); i++)
		{
			_output_ports[i]->output(
				position_ECI,
				attitude_ECI,
				velocity_ECI,
				angular_velocity,
				sun_dir_body,
				magnet_field_body
			);
		}
	}
}