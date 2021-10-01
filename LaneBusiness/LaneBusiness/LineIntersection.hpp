#pragma once
// The code adapted from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

class Point
{
public:
	double x;
	double y;

	void operator *= (const double& factor)
	{
		x *= factor;
		y *= factor;
	}

	void operator -= (const double& shift)
	{
		x -= shift;
		y -= shift;
	}

	void operator += (const double& shift)
	{
		x += shift;
		y += shift;
	}

	/// Rotates point in place counterclockwise
	void rotate(double angleRad)
	{
		auto point0 = x * sin(angleRad) + y * cos(angleRad);
		auto point1 = x * cos(angleRad) - y * sin(angleRad);

		x = point0;
		y = point1;
	}
};


// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
	int val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // collinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool areLinesIntersect(std::array<Point, 2>& line1, std::array<Point, 2>& line2)
{
	//Point p1; //line1[0]
	//Point q1; //line1[1]
	//Point p2; //line2[0]
	//Point q2; //line2[1]

	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(line1[0], line1[1], line2[0]);
	int o2 = orientation(line1[0], line1[1], line2[1]);
	int o3 = orientation(line2[0], line2[1], line1[0]);
	int o4 = orientation(line2[0], line2[1], line1[1]);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(line1[0], line2[0], line1[1])) return true;

	// p1, q1 and q2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(line1[0], line2[1], line1[1])) return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(line2[0], line1[0], line2[1])) return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(line2[0], line1[1], line2[1])) return true;

	return false; // Doesn't fall in any of the above cases
}
