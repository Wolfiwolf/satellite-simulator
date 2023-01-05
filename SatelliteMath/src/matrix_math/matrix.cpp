#include "matrix.hpp"

#include <cmath>

namespace sat_math
{
	Matrix::Matrix() :_m(0),_n(0) {}

	Matrix::Matrix(const Matrix& m) : 
		_m(m._m),
		_n(m._n)
	{
		
		for (int i = 0; i < m._m * m._n; i++)
		{
			_data[i] = m._data[i];
		}
	}

	Matrix::Matrix(const Matrix&& m) :
		_m(m._m),
		_n(m._n)
	{

		for (int i = 0; i < m._m * m._n; i++)
		{
			_data[i] = m._data[i];
		}
	}

	Matrix::Matrix(int M, int N) :
		_m(M),
		_n(N)
	{

		for (int i = 0; i < M * N; i++)
		{
			_data[i] = 0.0;
		}

	}

	Matrix::~Matrix()
	{

	}

	Matrix Matrix::add(const Matrix &m)
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m * _n; i++)
		{
			res._data[i] = _data[i] + m._data[i];
		}

		return res;
	}

	Matrix Matrix::add(const double& scalar)
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m * _n; i++)
		{
			res._data[i] = _data[i] + scalar;
		}

		return res;
	}

	Matrix Matrix::sub(const Matrix& m)
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m * _n; i++)
		{
			res._data[i] = _data[i] - m._data[i];
		}

		return res;
	}

	Matrix Matrix::sub(const double& scalar)
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m * _n; i++)
		{
			res._data[i] = _data[i] - scalar;
		}

		return res;
	}

	Matrix Matrix::transpose()
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m; i++)
		{
			for (int j = 0; j < _n; j++)
			{
				res(i, j) = (*this)(j, i);
			}
		}

		return res;
	}

	double Matrix::magnitude()
	{
		double sum = 0.0;

		for (int i = 0; i < _m * _n; i++)
		{
			sum += _data[i] * _data[i];
		}

		return sqrt(sum);
	}

	Matrix Matrix::multiply(const Matrix& m) const
	{
		Matrix res(_m, m._n);

		for (int i = 0; i < res._m; i++)
		{
			for (int j = 0; j < res._n; j++)
			{
				double sum = 0.0;

				for (int k = 0; k < _n; k++)
				{
					sum += (*this)(i, k) * m(k, j);
				}

				res(i, j) = sum;
			}
		}

		return res;
	}

	Matrix Matrix::multiply(const double& scalar) const
	{
		Matrix res(_m, _n);

		for (int i = 0; i < _m * _n; i++)
		{
			res._data[i] = _data[i] * scalar;
		}

		return res;
	}


	Matrix Matrix::cross(const Matrix& m)
	{
		Matrix res(_m, _n);

		res(0, 0) = (*this)(1, 0) * m(2, 0) - (*this)(2, 0) * m(1, 0);
		res(1, 0) = (*this)(2, 0) * m(0, 0) - (*this)(0, 0) * m(2, 0);
		res(2, 0) = (*this)(0, 0) * m(1, 0) - (*this)(1, 0) * m(0, 0);

		return res;
	}

	Matrix Matrix::normalize()
	{
		double magnitude = (*this).magnitude();
		if (magnitude == 0.0) return (*this);
		return (*this) * (1 / magnitude);
	}

	Matrix Matrix::operator = (const Matrix& m)
	{
		_m = m._m;
		_n = m._n;

		for (int i = 0; i < m._m * m._n; i++)
		{
			_data[i] = m._data[i];
		}

		return *(this);
	}

	Matrix Matrix::operator * (const Matrix& m) const
	{
		return (*this).multiply(m);
	}

	Matrix Matrix::operator * (const double& scalar) const
	{
		return (*this).multiply(scalar);
	}

	Matrix Matrix::operator + (const Matrix& m)
	{
		return (*this).add(m);
	}

	Matrix Matrix::operator + (const double& scalar)
	{
		return (*this).add(scalar);
	}

	Matrix Matrix::operator - (const Matrix& m)
	{
		return (*this).sub(m);
	}

	Matrix Matrix::operator - (const double& scalar)
	{
		return (*this).sub(scalar);
	}

	double& Matrix::operator () (int m, int n)
	{
		return _data[m * _n + n];
	}

	const double& Matrix::operator ()  (int m, int n) const
	{
		return _data[m * _n + n];
	}
}
