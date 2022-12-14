#include <iostream>
#include "satellite_math.hpp";
#include "satellite/satellite.hpp"
#include "satellite_simulator_data_outputer.hpp"

int main() 
{
	satellite_simulator::Satellite satellite;
	satellite_simulator_data_outputer::SatelliteSimulatorDataOutputer outputer;

	sat_math::Matrix position_ECI(3, 1);
	sat_math::Matrix  attitude_ECI(4, 1);
	sat_math::Matrix  velocity_ECI(3, 1);
	sat_math::Matrix  angular_velocity(3, 1);
	sat_math::Matrix  magnet_field_ECI(3, 1);
	sat_math::Matrix  magnet_field_body(3, 1);
	sat_math::Matrix  sun_dir_ECI(3, 1);
	sat_math::Matrix  sun_dir_body(3, 1);



	double time = 0.0;
	const double delta_time = 0.01;
	while (true)
	{
		bool is_ok = satellite.update_state(delta_time);

		
		if (!is_ok)
		{
			int delta_time_devider = 2;

			while (!is_ok)
			{
				double new_delta_time = delta_time / (double)delta_time_devider;
				is_ok = satellite.update_state(new_delta_time);
				if (is_ok)
				{
					for (int i = 1; i < delta_time_devider; i++)
					{
						satellite.update_state(new_delta_time);
					}
					break;
				}
				else
				{
					delta_time_devider *= 2;
				}
			}
		}


		sat_math::Matrix pos = satellite.get_ECI_position();
		sat_math::Matrix attitude = satellite.get_ECI_attitude();

		outputer.output(
			pos,
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