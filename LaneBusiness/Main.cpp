#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "WSG84toCartesian.hpp"

using namespace cv;
using namespace wgs84;


std::vector<int> busyLanes(std::array<double, 3> rPos, double rAzimuth,
	std::array<double, 3> aPos, std::array<double, 3> bPos, int nLanes, double laneWidth,
	std::array<double, 3> objPos, double objYaw, double objLength, double objWidth)
{
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

	if(result == std::vector<int>({1, 2}))
	return true;
}


int main()
{
	assert(test());
	return 0;
}
