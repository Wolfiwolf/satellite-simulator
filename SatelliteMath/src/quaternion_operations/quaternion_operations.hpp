#pragma once

#include "../matrix_math/matrix.hpp"

namespace sat_math
{

	static class QuaternionOperations
	{
	public:
		static Matrix quaternion_to_euler_angles_ZYX(const Matrix& quaternion);
		static Matrix quaternion_dot_from_angular_velocity(const Matrix& quaternion, const Matrix &angular_velocities);
	};

}