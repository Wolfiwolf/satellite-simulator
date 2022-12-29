#pragma once

#include "../matrix_math/matrix.hpp"

namespace sat_math
{

	static class VectorTransformations
	{
	public:
		static Matrix ECI_to_body(const Matrix &vec_ECI);
		static Matrix body_to_ECI(const Matrix& vec_body);
	};

}