#include "space_operations.hpp"

#include <cmath>

namespace satellite_simulator_engine
{
	const unsigned long long int SpaceOperations::_G = 398589200000000;
	const double SpaceOperations::_OMEGA = 7.29211509e-5;
	const GeographicLib::GravityModel SpaceOperations::_gravity_model("wgs84");
	const GeographicLib::MagneticModel SpaceOperations::_magnetic_model("wmm2020");
	const GeographicLib::Geocentric SpaceOperations::_geo = GeographicLib::Geocentric::WGS84();
	
	sat_math::Matrix SpaceOperations::get_gravity_force_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI, const double& mass)
	{
		// position ECI -> ECEF
		sat_math::Matrix pos_ecef = ECI_to_ECEF(time_since_epoch, pos_ECI);

		// get gravity in ECEF
		sat_math::Matrix gravity_ECEF(3, 1);
		_gravity_model.V(pos_ecef(0, 0), pos_ecef(1, 0), pos_ecef(2, 0), gravity_ECEF(0, 0), gravity_ECEF(1, 0), gravity_ECEF(2, 0));

		// gravity ECEF -> ECI
		return ECEF_to_ECI(time_since_epoch, gravity_ECEF) * mass;
	}

	sat_math::Matrix SpaceOperations::get_magnet_field_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI)
	{
		// Mag Field: get field at that position in ENU frame, in nanoTesla
		sat_math::Matrix r_ECEF = ECI_to_ECEF(time_since_epoch, pos_ECI);

		// Position: ECEF -> WGS84
		double lat, lon, h;

		_geo.Reverse(r_ECEF(0, 0), r_ECEF(1, 0), r_ECEF(2, 0), lat, lon, h);


		// Mag Field: get field at that position in ENU frame, in nanoTesla
		sat_math::Matrix B_ENU(3, 1);
		_magnetic_model(2020, lat, lon, h, B_ENU(0, 0), B_ENU(1, 0), B_ENU(2, 0));
		

		// Mag Field: convert to Tesla
		B_ENU = B_ENU * 1e-9;

		// Mag Field: (rotation only) ENU -> ECEF
		sat_math::Matrix B_ECEF = ENU_to_ECEF(lat, lon, B_ENU);

		// Mag Field: ECEF -> ECI
		sat_math::Matrix B_ECI = ECEF_to_ECI(time_since_epoch, B_ECEF);


		return B_ECI;
	}

	sat_math::Matrix SpaceOperations::get_sun_direction_ECI(const double time_since_epoch, const sat_math::Matrix& pos_ECI)
	{
		sat_math::Matrix sun_ECI(3, 1);


		int day_in_year = ((int)(time_since_epoch) / 86400) % 365;

		double L = 280.4606184 + (36000.77005361 / 36525) * day_in_year;

		double g = 357.5277233 + ((35999.05034 / 36525) * day_in_year);

		constexpr double PI = 3.14159265358979323846;
		double p = L + (1.914666471 * sin(g * PI / 180)) + 0.918994643 * sin(2 * g * PI / 180);

		double q = 23.43929 - ((46.8093 / 3600) * (day_in_year / 36525));

		double x = cos(p * PI / 180);
		double y = cos(q * PI / 180) * sin(p * PI / 180);
		double z = sin(q * PI / 180) * sin(p * PI / 180);


		sun_ECI(0, 0) = x;
		sun_ECI(1, 0) = y;
		sun_ECI(2, 0) = z;

		return sun_ECI;
	}

	sat_math::Matrix SpaceOperations::ECI_to_ECEF(const double time_since_epoch, const sat_math::Matrix& vec_ECI) {
		double yaw = time_since_epoch * _OMEGA;
		double sin_yaw = sin(yaw), cos_yaw = cos(yaw);

		sat_math::Matrix ecef(3, 1);
		ecef(0, 0) = cos_yaw * vec_ECI(0, 0) + sin_yaw * vec_ECI(1, 0);
		ecef(1, 0) = -sin_yaw * vec_ECI(0, 0) + cos_yaw * vec_ECI(1, 0);
		ecef(2, 0) = vec_ECI(2, 0);

		return ecef;
	}


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