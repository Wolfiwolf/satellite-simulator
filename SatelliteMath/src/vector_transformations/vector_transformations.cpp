#pragma once

#include "vector_transformations.hpp"
#include <cmath>

namespace sat_math
{

	Matrix VectorTransformations::attitude_to_rotation_matrix(const Matrix& attitude_quaternion)
	{
		Matrix euler = QuaternionOperations::quaternion_to_euler_angles_ZYX(attitude_quaternion);

		const double phi = euler(0, 0);
		const double theta = euler(1, 0);
		const double psi = euler(2, 0);

		Matrix Rx(3, 3);

		Rx(0, 0) = 1.0;
		Rx(0, 1) = 0.0;
		Rx(0, 2) = 0.0;

		Rx(1, 0) = 0.0;
		Rx(1, 1) = cos(phi);
		Rx(1, 2) = -sin(phi);

		Rx(2, 0) = 0.0;
		Rx(2, 1) = sin(phi);
		Rx(2, 2) = cos(phi);

		Matrix Ry(3, 3);

		Ry(0, 0) = cos(theta);
		Ry(0, 1) = 0.0;
		Ry(0, 2) = sin(theta);

		Ry(1, 0) = 0.0;
		Ry(1, 1) = 1.0;
		Ry(1, 2) = 0.0;

		Ry(2, 0) = -sin(theta);
		Ry(2, 1) = 0.0;
		Ry(2, 2) = cos(theta);

		Matrix Rz(3, 3);

		Rz(0, 0) = cos(psi);
		Rz(0, 1) = -sin(psi);
		Rz(0, 2) = 0.0;

		Rz(1, 0) = sin(psi);
		Rz(1, 1) = cos(psi);
		Rz(1, 2) = 0.0;

		Rz(2, 0) = 0.0;
		Rz(2, 1) = 0.0;
		Rz(2, 2) = 1.0;


		return Rz * Ry * Rx;
	}

	Matrix VectorTransformations::ECI_to_body(const Matrix& vec_ECI, const Matrix& body_attitude_quaternion_ECI)
	{
		Matrix R = attitude_to_rotation_matrix(body_attitude_quaternion_ECI);

		return R.transpose() * vec_ECI;
	}

	Matrix VectorTransformations::body_to_ECI(const Matrix& vec_body, const Matrix& body_attitude_quaternion_ECI)
	{
		Matrix R = attitude_to_rotation_matrix(body_attitude_quaternion_ECI);

		return R * vec_body;
	}


}