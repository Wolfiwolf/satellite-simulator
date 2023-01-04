#pragma once

#include "satellite_math.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/GravityModel.hpp>
#include <GeographicLib/MagneticModel.hpp>

namespace satellite_simulator_engine
{

	static class SpaceOperations
	{
	public:
		static sat_math::Matrix get_gravity_force_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI, const double& mass);
		static sat_math::Matrix get_magnet_field_ECI(const double time_since_epoch, const sat_math::Matrix &pos_ECI);
		static sat_math::Matrix get_sun_direction_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI);

		static sat_math::Matrix ECI_to_ECEF(const double time_since_epoch, const sat_math::Matrix& vec_ECI);
		static sat_math::Matrix ECEF_to_ECI(const double time_since_epoch, const sat_math::Matrix& vec_ECEF);
		static sat_math::Matrix ENU_to_ECEF(const double lat_deg, const double lon_deg, const sat_math::Matrix& vec_ENU);

	private:
		static const unsigned long long int _G;
		static const unsigned long long int _OMEGA;
		static const GeographicLib::GravityModel _gravity_model;
		static const GeographicLib::MagneticModel _magnetic_model;
		static const GeographicLib::Geocentric _geo;

	};
}