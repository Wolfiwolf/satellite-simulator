#include "file_output_port.hpp"

#include <iostream>

namespace satellite_simulator_data_outputer
{
	FileOutputPort::FileOutputPort(const std::string& directory)
	{
		_output_files["position_ECI"] = std::ofstream(directory + "position_ECI.txt");
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
		_output_files["position_ECI"] << position_ECI(0, 0) << " " << position_ECI(1, 0) << " " << position_ECI(2, 0) << '\n';
	}

}