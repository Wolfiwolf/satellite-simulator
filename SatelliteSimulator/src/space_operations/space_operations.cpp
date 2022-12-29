#include "space_operations.hpp"

#include <cmath>

namespace satellite_simulator_engine
{
	const unsigned long long int SpaceOperations::_G = 398589200000000;
	const unsigned long long int SpaceOperations::_OMEGA = 398589200000000;
	const GeographicLib::GravityModel SpaceOperations::_gravity_model("wgs84");
	const GeographicLib::MagneticModel SpaceOperations::_magnetic_model("wmm2020");
	const GeographicLib::Geocentric SpaceOperations::_geo = GeographicLib::Geocentric::WGS84();
	
	sat_math::Matrix SpaceOperations::get_gravity_force(const double time_since_epoch, const sat_math::Matrix& pos_ECI, const double& mass)
	{
		/*
		double tempG = _G * mass;
		double tempr = r * r;
		return tempG / tempr;
		*/
		// position ECI -> ECEF
		sat_math::Matrix pos_ecef = ECI_to_ECEF(time_since_epoch, pos_ECI);

		// get gravity in ECEF
		sat_math::Matrix gravity_ECEF(3, 1);
		_gravity_model.V(pos_ecef(0, 0), pos_ecef(1, 0), pos_ecef(2, 0), gravity_ECEF(0, 0), gravity_ECEF(1, 0), gravity_ECEF(2, 0));

		// gravity ECEF -> ECI
		return ECEF_to_ECI(time_since_epoch, gravity_ECEF) * mass;
	}

	sat_math::Matrix SpaceOperations::pos_ECI_to_magnet_field_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI)
	{
		// Mag Field: get field at that position in ENU frame, in nanoTesla
		sat_math::Matrix r_ECEF = ECI_to_ECEF(time_since_epoch, pos_ECI);


		// Position: ECEF -> WGS84
		double lat = 0.0, lon = 0.0, h = 0.0;

		_geo.Reverse(r_ECEF(0, 0), r_ECEF(1, 0), r_ECEF(2, 0), lat, lon, h);


		// Mag Field: get field at that position in ENU frame, in nanoTesla
		sat_math::Matrix B_ENU(3, 1);
		_magnetic_model(2016, lat, lon, h, B_ENU(0, 0), B_ENU(1, 0), B_ENU(2, 0));

		// Mag Field: convert to Tesla
		B_ENU = B_ENU * 1e-9;

		// Mag Field: (rotation only) ENU -> ECEF
		sat_math::Matrix B_ECEF = ENU_to_ECEF(lat, lon, B_ENU);

		// Mag Field: ECEF -> ECI
		sat_math::Matrix B_ECI = ECEF_to_ECI(time_since_epoch, B_ECEF);


		return B_ECI;
	}

	// convert a vector in the ECI frame to the ECEF frame
	// the two frames are identical when t = 0
	sat_math::Matrix SpaceOperations::ECI_to_ECEF(const double time_since_epoch, const sat_math::Matrix& vec_ECI) {
		double yaw = time_since_epoch * _OMEGA;
		double sin_yaw = sin(yaw), cos_yaw = cos(yaw);

		sat_math::Matrix ecef(3, 1);
		ecef(0, 0) = cos_yaw * vec_ECI(0, 0) + sin_yaw * vec_ECI(1, 0);
		ecef(1, 0) = -sin_yaw * vec_ECI(0, 0) + cos_yaw * vec_ECI(1, 0);
		ecef(2, 0) = vec_ECI(2, 0);

		return ecef;
	}

	// convert a vector in the ECEF frame to the ECI frame
	// the two frames are identical when t = 0
	sat_math::Matrix SpaceOperations::ECEF_to_ECI(const double time_since_epoch, const sat_math::Matrix& vec_ECEF) {
		double yaw = time_since_epoch * _OMEGA;
		double sin_yaw = sin(yaw), cos_yaw = cos(yaw);

		sat_math::Matrix eci(3, 1);
		eci(0, 0) = cos_yaw * vec_ECEF(0, 0) - sin_yaw * vec_ECEF(1, 0);
		eci(1, 0) = sin_yaw * vec_ECEF(0, 0) + cos_yaw * vec_ECEF(1, 0);
		eci(2, 0) = vec_ECEF(2, 0);

		return eci;
	}

	sat_math::Matrix SpaceOperations::ENU_to_ECEF(const double lat_deg, const double lon_deg, const sat_math::Matrix& enu) {
		double lat = lat_deg * (3.141592653589 / 180.0);
		double lon = lon_deg * (3.141592653589 / 180.0);
		double sin_lat = sin(lat);
		double cos_lat = cos(lat);
		double sin_long = sin(lon);
		double cos_long = cos(lon);
		double precomp = cos_lat * enu(2, 0) - sin_lat * enu(1, 0);

		sat_math::Matrix ecef(3, 1);
		ecef(0, 0) = cos_long * precomp - sin_long * enu(0, 0);
		ecef(1, 0) = sin_long * precomp + cos_long * enu(0, 0);
		ecef(2, 0) = cos_lat * enu(1, 0) + sin_lat * enu(2, 0);
		return ecef;
	}

}