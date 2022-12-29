#pragma once

#include "../matrix_math/matrix.hpp"
#include "../quaternion_operations/quaternion_operations.hpp"

namespace sat_math
{

	static class VectorTransformations
	{
	public:
		static Matrix attitude_to_rotation_matrix(const Matrix& attitude_quaternion);
		static Matrix ECI_to_body(const Matrix &vec_ECI, const Matrix& body_attitude_quaternion_ECI);
		static Matrix body_to_ECI(const Matrix& vec_body, const Matrix& body_attitude_quaternion_ECI);
	};

}