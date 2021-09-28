#pragma once
// The code adapted from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/


// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(std::array<double, 3> p, std::array<double, 3> q, std::array<double, 3> r)
{
	if (q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]) &&
		q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]))
		return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(std::array<double, 3> p, std::array<double, 3> q, std::array<double, 3> r)
{
	int val = (q[0] - p[0]) * (r[1] - q[1]) -
		(q[1] - p[1]) * (r[0] - q[0]);

	if (val == 0) return 0;  // collinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool areLinesIntersect(std::array<std::array<double, 3>, 2> line1, std::array<std::array<double, 3>, 2> line2)
{
	//std::array<double, 3> p1; //line1[0]
	//std::array<double, 3> q1; //line1[1]
	//std::array<double, 3> p2; //line2[0]
	//std::array<double, 3> q2; //line2[1]

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
