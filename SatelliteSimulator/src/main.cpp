#include <iostream>
#include "satellite_math.hpp";
#include "satellite_graphics_engine.hpp"
#include "satellite/satellite.hpp"

int main() 
{
	satellite_graphics_engine::SatelliteGraphicsEngine sat_graphics_engine;


	sat_graphics_engine.init();
	sat_graphics_engine.start();
	

	satellite_simulator::Satellite satellite;


	double time = 0.0;
	const double delta_time = 0.001;
	while (!sat_graphics_engine.should_program_end())
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

		sat_graphics_engine.set_satellite_state(
			pos(0, 0), 
			pos(1, 0), 
			pos(2, 0), 
			attitude(0, 0), 
			attitude(1, 0), 
			attitude(2, 0), 
			attitude(3, 0)
		);

		sat_graphics_engine.update();

		time += delta_time;
	}

	return 0;
}