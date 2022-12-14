#pragma once

#include "satellite_math.hpp"
#include "../output_port.hpp"
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>

namespace satellite_simulator_data_outputer
{

	class FileOutputPort : public OutputPort
	{
	public:
		FileOutputPort(const std::string &directory);

		virtual void output(const sat_math::Matrix& position_ECI,
			const sat_math::Matrix& attitude_ECI,
			const sat_math::Matrix& velocity_ECI,
			const sat_math::Matrix& angular_velocity,
			const sat_math::Matrix& magnet_field_ECI,
			const sat_math::Matrix& magnet_field_body,
			const sat_math::Matrix& sun_dir_ECI,
			const sat_math::Matrix& sun_dir_body);

	private:
		std::unordered_map<const char*, std::ofstream> _output_files;
	};

}
