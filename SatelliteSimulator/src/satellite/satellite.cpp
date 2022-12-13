#include "satellite.hpp"

#include "../space_operations/space_operations.hpp"

namespace satellite_simulator
{

	Satellite::Satellite()
	{
		_mass = 4.0;

		_I = sat_math::Matrix(3, 3);
		_I(0, 0) = 1.0f;
		_I(1, 1) = 1.0f;
		_I(2, 2) = 1.0f;

		_I_inverse = sat_math::Matrix(3, 3);
		_I_inverse(0, 0) = 1.0f;
		_I_inverse(1, 1) = 1.0f;
		_I_inverse(2, 2) = 1.0f;


		_position_ECI = sat_math::Matrix(3, 1);
		_position_ECI(0, 0) = 0.0f;
		_position_ECI(1, 0) = 0.0f;
		_position_ECI(2, 0) = 0.0f;

		_attitude_ECI = sat_math::Matrix(4, 1);
		_attitude_ECI(0, 0) = 1.0f;
		_attitude_ECI(1, 0) = 0.0f;
		_attitude_ECI(2, 0) = 0.0f;
		_attitude_ECI(3, 0) = 0.0f;

		_velocity_ECI = sat_math::Matrix(3, 1);
		_velocity_ECI(0, 0) = 0.0f;
		_velocity_ECI(1, 0) = 0.0f;
		_velocity_ECI(2, 0) = 0.0f;

		_angular_velocity = sat_math::Matrix(3, 1);
		_angular_velocity(0, 0) = 0.0f;
		_angular_velocity(1, 0) = 0.0f;
		_angular_velocity(2, 0) = 0.0f;

		_magnetorquers = sat_math::Matrix(3, 1);

	}

	Satellite::~Satellite()
	{

	}

	const sat_math::Matrix& Satellite::get_ECI_position() const
	{
		return _position_ECI;
	}

	const sat_math::Matrix& Satellite::get_ECI_attitude() const
	{
		return _attitude_ECI;
	}

	bool Satellite::update_state(const double& delta_time)
	{
		_update_velocity(delta_time);
		_update_angular_velocity(delta_time);
		_update_attitude(delta_time);
		_update_position(delta_time);

		return true;
	}

	void Satellite::_update_position(const double& delta_time)
	{
		_position_ECI = _position_ECI + (_velocity_ECI * delta_time);
	}

	void Satellite::_update_velocity(const double& delta_time)
	{
		sat_math::Matrix forces(3, 1);

		sat_math::Matrix gravity_vector(3, 1);
		gravity_vector = _position_ECI * -1;

		gravity_vector = gravity_vector.normalize();

		gravity_vector = gravity_vector * SpaceOperations::get_gravity_force(_mass, _position_ECI.magnitude());

		forces = forces + gravity_vector;

		sat_math::Matrix velocity_dot(3, 1);

		velocity_dot = forces * (1 / _mass);

		_velocity_ECI = _velocity_ECI + (velocity_dot * delta_time);
	}

	void Satellite::_update_attitude(const double& delta_time)
	{
		sat_math::Matrix q_dot = sat_math::QuaternionOperations::quaternion_dot_from_angular_velocity(_attitude_ECI, _angular_velocity);

		_attitude_ECI = (_attitude_ECI + q_dot) * delta_time;
	}

	void Satellite::_update_angular_velocity(const double& delta_time)
	{
		sat_math::Matrix M(3, 1);
		
		M = _magnetorquers.cross(SpaceOperations::pos_ECI_to_magnet_field_ECI(_position_ECI));

		// TODO: ADD GRAVITY GRADIENT MOMENTS

		_angular_velocity = _angular_velocity +  (_I_inverse * (M - _angular_velocity.cross(_I * _angular_velocity)) * delta_time);
	}

}