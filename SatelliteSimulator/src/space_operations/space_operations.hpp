#pragma once

#include "satellite_math.hpp"

namespace satellite_simulator
{

	static class SpaceOperations
	{
	public:
		static double get_gravity_force(const double& mass, const double& r);
		static sat_math::Matrix pos_ECI_to_magnet_field_ECI(const sat_math::Matrix &pos_ECI);

	private:
		static const unsigned int _G;

	};
}