#include "file_output_port.hpp"

#include <iostream>

namespace satellite_simulator_data_outputer
{
	FileOutputPort::FileOutputPort(const std::string& directory)
	{
		
	}

	void FileOutputPort::output(
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
		std::cout << position_ECI(0, 0) << "\n";
	}

}