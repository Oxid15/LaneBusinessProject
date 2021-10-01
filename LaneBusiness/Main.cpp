#include "LaneBusiness\LaneBusiness.hpp"

bool test()
{
	auto rPos = Point({ 59.96769, 30.30985 }); // lat, long
	double rAzimuth = 30;

	auto aPos = Point({ 59.967681024726275, 30.310086939745639 });
	auto bPos = Point({ 59.967752830076989, 30.310118554680990 });
	uint32_t nLanes = 4;
	double laneWidth = 3.;

	auto objPos = Point({4., -3.}); // x, y
	double objYaw = 0.5235983;
	double objLength = 4.;
	double objWidth = 2.;

	auto result = busyLanes(rPos, rAzimuth, aPos, bPos, nLanes, laneWidth, objPos, objYaw, objLength, objWidth);

	if(result == std::vector<uint32_t>({4}))
		return true;
	else
		return false;
}

int main()
{
	assert(test());

	return 0;
}