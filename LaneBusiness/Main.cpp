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


std::vector<std::array<std::array<double, 3>, 4>> lanesCoordinates(std::array<double, 3> aPos, std::array<double, 3> bPos, double laneWidth, uint32_t nLanes)
{
	double roadYawRad = atan((aPos[0] - aPos[0]) / (bPos[1] - aPos[1]));

	double shiftY = aPos[0];
	double shiftX = aPos[1];

	aPos[0] -= shiftY;
	aPos[1] -= shiftX;
	bPos[0] -= shiftY;
	bPos[1] -= shiftX;

	rotate(aPos, -roadYawRad);
	rotate(bPos, -roadYawRad);

	auto lanesCoordinates = std::vector<std::array<std::array<double, 3>, 4>>();

	uint32_t i;
	double y;
	for (i = 0, y = -(nLanes * laneWidth / 2); i < nLanes; i++, y += laneWidth)
	{
		auto laneCoords = std::array<std::array<double, 3>, 4 >({
			std::array<double, 3>({y, 0., 0.}),                  // upper left
			std::array<double, 3>({y, bPos[1], 0.}),             // upper right
			std::array<double, 3>({y + laneWidth, bPos[1], 0.}), // lower right
			std::array<double, 3>({y + laneWidth, 0., 0.})       // lower left
			});

		for (auto& laneCoord : laneCoords)
		{
			laneCoord[0] += shiftY;
			laneCoord[1] += shiftX;
			rotate(laneCoord, roadYawRad);
		}
		lanesCoordinates.push_back(laneCoords);
	}

	return lanesCoordinates;
}

std::vector<int> busyLanes(std::array<double, 3> rPos, double rAzimuth,
	std::array<double, 3> aPos, std::array<double, 3> bPos, int nLanes, double laneWidth,
	std::array<double, 3> objPos, double objYaw, double objLength, double objWidth)
{
	// Find all the points of an object 
	auto objPoints = getBoxPoints(std::array<double, 3>({0., 0., 0.}), objWidth, objLength);
	for (auto &objPoint : objPoints)
	{
		rotate(objPoint, -objYaw);
		objPoint[0] += objPos[0];
		objPoint[1] += objPos[1];
	}

	// Find the road points in cartesian coords
	std::array<double, 3> aPosCart = toCartesian(rPos, aPos);
	std::array<double, 3> bPosCart = toCartesian(rPos, bPos);

	// Rotate them to match robot's coordinate system
	rotate(aPosCart, deg2rad(-rAzimuth));
	rotate(bPosCart, deg2rad(-rAzimuth));

	// Find all the points of the lanes

	auto lanePoints = lanesCoordinates(aPosCart, bPosCart, laneWidth, nLanes);
	
	// Visualize the scene in the coordinates of a robot
	visualizeScene(lanePoints, objPoints);

	// Check if any points of the object is inside any lane

	return std::vector<int>();
}

bool test()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. }); // lat, long, height
	double rAzimuth = 30;

	auto aPos = std::array<double, 3>({ 59.967681024726275, 30.310046939745639, 0. });
	auto bPos = std::array<double, 3>({ 59.967752830076989, 30.310118554680990, 0. });
	uint32_t nLanes = 2;
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