#pragma once
// The code adapted from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
#include "Point.hpp"


/// Given three collinear points p, q, r, the function checks if
/// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r);

/// To find orientation of ordered triplet (p, q, r).
/// The function returns following values
/// 0 --> p, q and r are collinear
/// 1 --> Clockwise
/// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r);

bool areLinesIntersect(std::array<Point, 2>& line1, std::array<Point, 2>& line2);
