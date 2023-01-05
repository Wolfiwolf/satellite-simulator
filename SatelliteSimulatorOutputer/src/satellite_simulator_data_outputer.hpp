#pragma once

#include "satellite_math.hpp"
#include "output_ports/output_port.hpp"
#include <vector>
#include <string>


namespace satellite_simulator_data_outputer
{

	class SatelliteSimulatorDataOutputer
	{
	public:
		SatelliteSimulatorDataOutputer(const std::string& output_directory, const std::string& ip_address, int port);
		~SatelliteSimulatorDataOutputer();

		void output(
			const sat_math::Matrix& position_ECI,
			const sat_math::Matrix& attitude_ECI,
			const sat_math::Matrix& velocity_ECI,
			const sat_math::Matrix& angular_velocity,
			const sat_math::Matrix& sun_dir_body,
			const sat_math::Matrix& magnet_field_body
		);

	private:
		std::vector<OutputPort*> _output_ports;
	};

}