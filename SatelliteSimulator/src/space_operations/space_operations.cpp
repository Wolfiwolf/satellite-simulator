#include "space_operations.hpp"


namespace satellite_simulator
{
	const unsigned int SpaceOperations::_G = 398589200000000;
	
	double SpaceOperations::get_gravity_force(const double &mass, const double &r)
	{
		return (_G * mass ) / (r * r);
	}

	sat_math::Matrix SpaceOperations::pos_ECI_to_magnet_field_ECI(const sat_math::Matrix& pos_ECI)
	{
		sat_math::Matrix mag_field(3, 1);
		mag_field(2, 0) = -3700e-9;

		return mag_field;
	}

}