#include "LaneBusiness\LaneBusiness.hpp"

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

	if(result == std::vector<uint32_t>({1}))
		return true;
	else
		return false;
}

bool test2()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. }); // lat, long, height
	double rAzimuth = 30;

	auto aPos = std::array<double, 3>({ 59.967681024726275, 30.310046939745639, 0. });
	auto bPos = std::array<double, 3>({ 59.967752830076989, 30.310118554680990, 0. });
	uint32_t nLanes = 2;
	double laneWidth = 2.;

	auto objPos = std::array<double, 3>({ 3., 9., 0. }); // y, x, z
	double objYaw = 0.3235983;
	double objLength = 4.;
	double objWidth = 2.;

	auto result = busyLanes(rPos, rAzimuth, aPos, bPos, nLanes, laneWidth, objPos, objYaw, objLength, objWidth);

	if (result == std::vector<uint32_t>())
		return true;
	else
		return false;
}

bool test3()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. }); // lat, long, height
	double rAzimuth = 20;

	auto aPos = std::array<double, 3>({ 59.967681024726275, 30.310046939745639, 0. });
	auto bPos = std::array<double, 3>({ 59.967752830076989, 30.310118554680990, 0. });
	uint32_t nLanes = 6;
	double laneWidth = 2.;

	auto objPos = std::array<double, 3>({ 7., 9., 0. }); // y, x, z
	double objYaw = 0.3235983;
	double objLength = 4.;
	double objWidth = 2.;

	auto result = busyLanes(rPos, rAzimuth, aPos, bPos, nLanes, laneWidth, objPos, objYaw, objLength, objWidth);

	if (result == std::vector<uint32_t>({ 1, 2 }))
		return true;
	else
		return false;
}

int main()
{
	assert(test());
	assert(test2());
	assert(test3());

	return 0;
}