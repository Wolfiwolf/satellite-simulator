#pragma once

#include "satellite_math.hpp"

namespace satellite_simulator_engine
{

	class Satellite
	{
	public:
		Satellite();
		~Satellite();

		bool update_state(const double time_since_epoch, const double delta_time);
		void set_magnetourquers(double x, double y, double z);


		const sat_math::Matrix& get_ECI_position() const;
		const sat_math::Matrix& get_ECI_attitude() const;
		const sat_math::Matrix& get_angular_velocity() const;
		const sat_math::Matrix& get_sun_dir_body() const;
		const sat_math::Matrix& get_magnet_field_dir_body() const;

	private:
		double _mass;
		sat_math::Matrix _I;
		sat_math::Matrix _I_inverse;
		sat_math::Matrix _position_ECI;
		sat_math::Matrix _attitude_ECI;
		sat_math::Matrix _velocity_ECI;
		sat_math::Matrix _angular_velocity;

		sat_math::Matrix _magnet_field_dir_body;
		sat_math::Matrix _sun_dir_body;

		sat_math::Matrix _magnetorquers;

		
		void _update_position(const double delta_time);
		void _update_velocity(const double time_since_epoch, const double delta_time);
		void _update_attitude(const double delta_time);
		void _update_angular_velocity(const double time_since_epoch, const double delta_time);
		void _update_sun_dir_body(const double time_since_epoch, const double delta_time);
		void _update_magnet_field_dir_body(const double time_since_epoch, const double delta_time);
	};

}