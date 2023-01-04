#include "satellite.hpp"

#include "../space_operations/space_operations.hpp"


namespace satellite_simulator_engine
{

	Satellite::Satellite()
	{
		_mass = 4.0;

		_I = sat_math::Matrix( 3, 3 );
		_I(0, 0) = 1.0;
		_I(1, 1) = 1.0;
		_I(2, 2) = 1.0;

		_I_inverse = sat_math::Matrix( 3, 3 );
		_I_inverse(0, 0) = 1.0;
		_I_inverse(1, 1) = 1.0;
		_I_inverse(2, 2) = 1.0;

		_position_ECI = sat_math::Matrix( 3, 1 );
		_position_ECI(0, 0) = 12212000.0;
		_position_ECI(1, 0) = 0.0;
		_position_ECI(2, 0) = 0.0;

		_attitude_ECI = sat_math::Matrix( 4, 1 );
		_attitude_ECI(0, 0) = 1.0;
		_attitude_ECI(1, 0) = 0.0;
		_attitude_ECI(2, 0) = 0.0;
		_attitude_ECI(3, 0) = 0.0;

		double speed = 5714.0;

		_velocity_ECI = sat_math::Matrix( 3, 1 );
		_velocity_ECI(0, 0) = 0.0;
		_velocity_ECI(1, 0) = 1.0;
		_velocity_ECI(2, 0) = 0.0;

		_velocity_ECI = _velocity_ECI.normalize() * speed;


		_angular_velocity = sat_math::Matrix( 3, 1 );
		_angular_velocity(0, 0) = 0.04;
		_angular_velocity(1, 0) = 0.02;
		_angular_velocity(2, 0) = 0.005;

		_sun_dir_body = sat_math::Matrix(3, 1);

		_magnet_field_dir_body = sat_math::Matrix(3, 1);

		_magnetorquers = sat_math::Matrix( 3, 1 );

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

	const sat_math::Matrix& Satellite::get_angular_velocity() const
	{
		return _angular_velocity;
	}

	const sat_math::Matrix& Satellite::get_sun_dir_body() const
	{
		return _sun_dir_body;
	}

	const sat_math::Matrix& Satellite::get_magnet_field_dir_body() const
	{
		return _magnet_field_dir_body;
	}

	bool Satellite::update_state(const double time_since_epoch, const double delta_time)
	{
		_update_velocity(time_since_epoch, delta_time);
		_update_angular_velocity(time_since_epoch, delta_time);
		_update_attitude(delta_time);
		_update_position(delta_time);

		_update_sun_dir_body(delta_time);
		_update_magnet_field_dir_body(delta_time);
		

		return true;
	}

	void Satellite::_update_position(const double delta_time)
	{
		_position_ECI = _position_ECI + (_velocity_ECI * delta_time);
	}

	void Satellite::_update_velocity(const double time_since_epoch, const double delta_time)
	{
		sat_math::Matrix forces(3, 1);

		sat_math::Matrix gravity_vector(3, 1);
		gravity_vector = _position_ECI * -1;

		gravity_vector = gravity_vector.normalize();

		double r = _position_ECI.magnitude();

		sat_math::Matrix gravity_force = SpaceOperations::get_gravity_force_ECI(time_since_epoch, _position_ECI, _mass);

		forces = forces + gravity_force;

		sat_math::Matrix velocity_dot(3, 1);

		velocity_dot = forces * (1 / _mass);

		_velocity_ECI = _velocity_ECI + (velocity_dot * delta_time);
	}

	void Satellite::_update_attitude(const double delta_time)
	{
		sat_math::Matrix q_dot = sat_math::QuaternionOperations::quaternion_dot_from_angular_velocity(_attitude_ECI, _angular_velocity);

		_attitude_ECI = _attitude_ECI + (q_dot * delta_time);
		_attitude_ECI = _attitude_ECI.normalize();
	}

	void Satellite::_update_angular_velocity(const double time_since_epoch, const double delta_time)
	{
		sat_math::Matrix M(3, 1);

		_magnetorquers = _magnet_field_dir_body.cross(_angular_velocity) * -300000;
		
		M = _magnetorquers.cross(_magnet_field_dir_body);

		// TODO: ADD GRAVITY GRADIENT MOMENTS

		_angular_velocity = _angular_velocity +  ((_I_inverse * (M - _angular_velocity.cross(_I * _angular_velocity))) * delta_time);
	}

	void Satellite::_update_sun_dir_body(const double time_since_epoch, const double delta_time)
	{
		sat_math::Matrix sun_dir_ECI = SpaceOperations::get_sun_direction_ECI(time_since_epoch, _position_ECI);
		_sun_dir_body = sat_math::VectorTransformations::ECI_to_body(sun_dir_ECI, _attitude_ECI);
	}

	void Satellite::_update_magnet_field_dir_body(const double time_since_epoch, const double delta_time)
	{
		sat_math::Matrix B_ECI = SpaceOperations::get_magnet_field_ECI(time_since_epoch, _position_ECI);
		_magnet_field_dir_body = sat_math::VectorTransformations::ECI_to_body(B_ECI, _attitude_ECI);
	}
}