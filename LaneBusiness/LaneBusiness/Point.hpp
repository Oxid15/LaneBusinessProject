#include "WSG84toCartesian.hpp"


class Point
{
public:
	double x;
	double y;

	void operator *= (const double& factor);

	void operator -= (const double& shift);

	void operator += (const double& shift);

	/// Rotates point in place counterclockwise
	void rotate(double angleRad);
};
