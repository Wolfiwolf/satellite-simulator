#include "quaternion_operations.hpp"

#include <cmath>

namespace sat_math
{

	Matrix QuaternionOperations::quaternion_to_euler_angles_ZYX(const Matrix& quaternion)
	{
		double qw = quaternion(0, 0);
		double qx = quaternion(1, 0);
		double qy = quaternion(2, 0);
		double qz = quaternion(3, 0);

		double qw2 = qw * qw;
		double qx2 = qx * qx;
		double qy2 = qy * qy;
		double qz2 = qz * qz;


		Matrix euler_angles(3, 1);

		double C11 = qw2 + qx2 - qy2 - qz2;
		double C12 = 2 * (qx * qy + qz * qw);
		double C13 = 2 * (qx * qz - qy * qw);
		double C23 = 2 * (qy * qz + qx * qw);
		double C33 = qw2 - qx2 - qy2 + qz2;


		euler_angles(0, 0) = atan2(C23, C33);
		euler_angles(1, 0) = -asin(C13);
		euler_angles(2, 0) = atan2(C12, C11);


		return euler_angles;
	}


	Matrix QuaternionOperations::quaternion_dot_from_angular_velocity(const Matrix& quaternion, const Matrix& angular_velocities)
	{
		double wx = angular_velocities(0, 0);
		double wy = angular_velocities(1, 0);
		double wz = angular_velocities(2, 0);

		Matrix omega(4, 4);

		omega(0, 0) = 0.0;
		omega(0, 1) = -wx;
		omega(0, 2) = -wy;
		omega(0, 3) = -wz;

		omega(1, 0) = wx;
		omega(1, 1) = 0.0;
		omega(1, 2) = wz;
		omega(1, 3) = -wy;

		omega(2, 0) = wy;
		omega(2, 1) = -wz;
		omega(2, 2) = 0.0;
		omega(2, 3) = wx;

		omega(3, 0) = wz;
		omega(3, 1) = wy;
		omega(3, 2) = -wx;
		omega(3, 3) = 0.0;

		return omega * quaternion * 0.5;
	}

}