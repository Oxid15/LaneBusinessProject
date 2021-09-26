#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "WSG84toCartesian.hpp"

using namespace cv;

const double M_PI = 3.141592653589793;

double deg2rad(double deg) {
	return deg * M_PI / 180.0;
}


/// Returns 4 box points given center and width with height
std::array<std::array<double, 3>, 4> getBoxPoints(std::array<double, 3> centerPoint, double length, double height)
{
	std::array<double, 3> upperLeft = {centerPoint[0] + height / 2, centerPoint[1] - length / 2, 0.};
	std::array<double, 3> upperRight = { centerPoint[0] + height / 2, centerPoint[1] + length / 2, 0. };
	std::array<double, 3> lowerLeft = { centerPoint[0] - height / 2, centerPoint[1] - length / 2, 0. };
	std::array<double, 3> lowerRight = { centerPoint[0] - height / 2, centerPoint[1] + length / 2, 0. };

	return std::array<std::array<double, 3>, 4>({ upperLeft, upperRight, lowerLeft, lowerRight});
}


/// Rotates point in place counterclockwise
void rotate(std::array<double, 3>& point, double angleRad)
{
	auto point0 = point[1] * sin(angleRad) + point[0] * cos(angleRad); // y
	auto point1 = point[1] * cos(angleRad) - point[0] * sin(angleRad); // x

	point[0] = point0;
	point[1] = point1;
}


/// Wrapper around similar function from namespace wgs84 which is 2DoF
std::array<double, 3> toCartesian(std::array<double, 3> referencePoint, std::array<double, 3> targetPoint)
{
	std::array<double, 2> refPoint2DoF;
	std::array<double, 2> targetPoint2DoF;
	std::copy(referencePoint.begin(), referencePoint.begin() + 2, refPoint2DoF.begin());
	std::copy(targetPoint.begin(), targetPoint.begin() + 2, targetPoint2DoF.begin());

	std::array<double, 2> result2Dof = wgs84::toCartesian(refPoint2DoF, targetPoint2DoF);

	std::array<double, 3> result = { 0., 0., 0. };
	std::copy(result2Dof.begin(), result2Dof.begin() + 2, result.begin());

	return result;
}


std::vector<int> busyLanes(std::array<double, 3> rPos, double rAzimuth,
	std::array<double, 3> aPos, std::array<double, 3> bPos, int nLanes, double laneWidth,
	std::array<double, 3> objPos, double objYaw, double objLength, double objWidth)
{
	//auto debug_result = wgs84::toCartesian(std::array<double, 2>({ 59.96769, 30.30985}), std::array<double, 2>({ 59.96783, 30.30958}));

	// Find the road points in cartesian coords
	std::array<double, 3> aPosCart = toCartesian(rPos, aPos);
	std::array<double, 3> bPosCart = toCartesian(rPos, bPos);

	// Rotate them to match robot's coordinate system
	rotate(aPosCart, deg2rad(-rAzimuth));
	rotate(bPosCart, deg2rad(-rAzimuth));

	// Find all the points of an object 
	// objPos
	auto objPoints = getBoxPoints(std::array<double, 3>({0., 0., 0.}), objWidth, objLength);
	for (auto &objPoint : objPoints)
	{
		rotate(objPoint, -objYaw);
		objPoint[0] += objPos[0];
		objPoint[1] += objPos[1];
	}

	// Find all the points of the Lanes

	// Check if any points of the object is inside any lane

	return std::vector<int>(1, 2);
}

bool test()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. }); // lat, long, height
	double rAzimuth = 30;

	auto aPos = std::array<double, 3>({ 59.967681024726275, 30.310046939745639, 0. });
	auto bPos = std::array<double, 3>({ 59.967752830076989, 30.310118554680990, 0. });
	int nLanes = 2;
	double laneWidth = 2.;

	auto objPos = std::array<double, 3>({7., 9., 0.}); // y, x, z
	double objYaw = 0.5235983;
	double objLength = 4.;
	double objWidth = 2.;

	auto result = busyLanes(rPos, rAzimuth, aPos, bPos, nLanes, laneWidth, objPos, objYaw, objLength, objWidth);

	//if(result == std::vector<int>({1, 2}))
	return true;
}


int main()
{
	assert(test());
	return 0;
}
