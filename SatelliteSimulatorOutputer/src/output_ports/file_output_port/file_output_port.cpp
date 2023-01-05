#include "file_output_port.hpp"

#include <iostream>

namespace satellite_simulator_data_outputer
{
	FileOutputPort::FileOutputPort(const std::string& directory)
	{

		_output_files["position_ECI"] = std::ofstream(directory + "position_ECI.txt");
		_output_files["attitude_euler_angles_ECI"] = std::ofstream(directory + "attitude_euler_angles_ECI.txt");
		_output_files["angular_velocity"] = std::ofstream(directory + "angular_velocity.txt");
		_output_files["sun_body"] = std::ofstream(directory + "sun_body.txt");
		_output_files["magnet_field_body"] = std::ofstream(directory + "magnet_field_body.txt");

	}

	void FileOutputPort::output(
		const sat_math::Matrix& position_ECI,
		const sat_math::Matrix& attitude_ECI,
		const sat_math::Matrix& velocity_ECI,
		const sat_math::Matrix& angular_velocity,
		const sat_math::Matrix& sun_dir_body,
		const sat_math::Matrix& magnet_field_body
	)
	{
		_output_files["position_ECI"] << position_ECI(0, 0) << " " << position_ECI(1, 0) << " " << position_ECI(2, 0) << '\n';
		sat_math::Matrix euler_angles = sat_math::QuaternionOperations::quaternion_to_euler_angles_ZYX(attitude_ECI);
		_output_files["attitude_euler_angles_ECI"] << euler_angles(0, 0) << " " << euler_angles(1, 0) << " " << euler_angles(2, 0) << '\n';
		_output_files["angular_velocity"] << angular_velocity(0, 0) << " " << angular_velocity(1, 0) << " " << angular_velocity(2, 0) << '\n';
		_output_files["sun_body"] << sun_dir_body(0, 0) << " " << sun_dir_body(1, 0) << " " << sun_dir_body(2, 0) << '\n';
		_output_files["magnet_field_body"] << magnet_field_body(0, 0) << " " << magnet_field_body(1, 0) << " " << magnet_field_body(2, 0) << '\n';
	}

}