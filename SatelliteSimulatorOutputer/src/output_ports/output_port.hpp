#pragma once

#include "satellite_math.hpp"

namespace satellite_simulator_data_outputer
{

	class OutputPort
	{
	public:
		virtual void output(
			const sat_math::Matrix& position_ECI,
			const sat_math::Matrix& attitude_ECI,
			const sat_math::Matrix& velocity_ECI,
			const sat_math::Matrix& angular_velocity,
			const sat_math::Matrix& magnet_field_ECI,
			const sat_math::Matrix& magnet_field_body,
			const sat_math::Matrix& sun_dir_ECI,
			const sat_math::Matrix& sun_dir_body
		) = 0;

	};

}