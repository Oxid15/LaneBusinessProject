#include "Point.hpp"

void Point::operator *= (const double& factor)
{
	x *= factor;
	y *= factor;
}

void Point::operator -= (const double& shift)
{
	x -= shift;
	y -= shift;
}

void Point::operator += (const double& shift)
{
	x += shift;
	y += shift;
}

/// Rotates point in place counterclockwise
void Point::rotate(double angleRad)
{
	auto point0 = x * sin(angleRad) + y * cos(angleRad);
	auto point1 = x * cos(angleRad) - y * sin(angleRad);

	x = point0;
	y = point1;
}