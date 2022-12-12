#pragma once

#include "satellite_math.hpp"

namespace satellite_simulator
{

	class Satellite
	{
	public:
		Satellite();
		~Satellite();

		bool update_state(const double& delta_time);
		void set_magnetourquers(double x, double y, double z);


		const sat_math::Matrix& get_ECI_position() const;
		const sat_math::Matrix& get_ECI_attitude() const;

	private:
		double _mass;
		sat_math::Matrix _I;
		sat_math::Matrix _I_inverse;
		sat_math::Matrix _position_ECI;
		sat_math::Matrix _attitude_ECI;
		sat_math::Matrix _velocity_ECI;
		sat_math::Matrix _angular_velocity;

		sat_math::Matrix _magnetorquers;

		
		void _update_position(const double& delta_time);
		void _update_velocity(const double& delta_time);
		void _update_attitude(const double& delta_time);
		void _update_angular_velocity(const double& delta_time);
	};

}