#pragma once


namespace sat_math
{

	class Matrix
	{
	public:
		Matrix();
		Matrix(const Matrix& m);
		Matrix(const Matrix&& m);
		Matrix(int M, int N);
		~Matrix();

		Matrix add(const Matrix& m);
		Matrix add(const double& scalar);
		Matrix sub(const Matrix& m);
		Matrix sub(const double& scalar);
		Matrix transpose();
		double magnitude();
		Matrix multiply(const Matrix& m) const;
		Matrix multiply(const double& scalar) const;
		Matrix cross(const Matrix& m);
		Matrix normalize();

		Matrix operator = (const Matrix& m);
		double& operator () (int m, int n);
		const double& operator ()  (int m, int n) const;
		Matrix operator * (const Matrix& m) const;
		Matrix operator * (const double& scalar) const;
		Matrix operator + (const Matrix& m);
		Matrix operator + (const double& scalar);
		Matrix operator - (const Matrix& m);
		Matrix operator - (const double& scalar);

	protected:
		int _m, _n;
		double _data[4 * 4];
	};

}