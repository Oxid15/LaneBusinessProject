#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "WSG84toCartesian.hpp"

using namespace cv;


std::array<double, 3> toCartesian(std::array<double, 3> referencePoint, std::array<double, 3> targetPoint)
{
	/// Wrapper around similar function from namespace wgs84 which is 2DoF

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

	// Find all the points of the Lanes

	// Find all the points of an object

	// Check if any points of the object is inside any lane

	return std::vector<int>(1, 2);
}

bool test()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. });
	double rAzimuth = 110.;

	auto aPos = std::array<double, 3>({ 59.96783, 30.30958, 0. });
	auto bPos = std::array<double, 3>({ 59.96804, 30.30938, 0. });
	int nLanes = 2;
	double laneWidth = 2.;

	auto objPos = std::array<double, 3>({-8., 17., 0.});
	double objYaw = 3.14;
	double objLength = 4.5;
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
