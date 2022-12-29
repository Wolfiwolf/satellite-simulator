#include <iostream>
#include "satellite_math.hpp";
#include "satellite/satellite.hpp"
#include "satellite_simulator_data_outputer.hpp"

int main(int argc, char *argv[])
{
	satellite_simulator_engine::Satellite satellite;
	satellite_simulator_data_outputer::SatelliteSimulatorDataOutputer outputer(argv[1], argv[2], std::stoi(argv[3]));

	sat_math::Matrix position_ECI(3, 1);
	sat_math::Matrix attitude_ECI(4, 1);
	sat_math::Matrix velocity_ECI(3, 1);
	sat_math::Matrix angular_velocity(3, 1);
	sat_math::Matrix magnet_field_ECI(3, 1);
	sat_math::Matrix magnet_field_body(3, 1);
	sat_math::Matrix sun_dir_ECI(3, 1);
	sat_math::Matrix sun_dir_body(3, 1);



	double time = 0.0;
	const double delta_time = 0.5;
	while (true)
	{
		bool is_ok = satellite.update_state(time, delta_time);

		
		if (!is_ok)
		{
			int delta_time_devider = 2;

			while (!is_ok)
			{
				double new_delta_time = delta_time / (double)delta_time_devider;
				is_ok = satellite.update_state(time, new_delta_time);
				if (is_ok)
				{
					for (int i = 1; i < delta_time_devider; i++)
					{
						satellite.update_state(time, new_delta_time);
					}
					break;
				}
				else
				{
					delta_time_devider *= 2;
				}
			}
		}


		position_ECI = satellite.get_ECI_position();
		attitude_ECI = satellite.get_ECI_attitude();
		angular_velocity = satellite.get_angular_velocity();

		outputer.output(
			position_ECI,
			attitude_ECI,
			velocity_ECI,
			angular_velocity,
			magnet_field_ECI,
			magnet_field_body,
			sun_dir_ECI,
			sun_dir_body
		);

		time += delta_time;
	}

	return 0;
}