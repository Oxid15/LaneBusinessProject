#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "WSG84toCartesian.hpp"

const double M_PI = 3.141592653589793;

double deg2rad(double deg) {
	return deg * M_PI / 180.0;
}

/// Returns 4 box points given center and width with height
std::array<std::array<double, 3>, 4> getBoxPoints(std::array<double, 3> centerPoint, double length, double height)
{
	std::array<double, 3> upperLeft = {centerPoint[0] + height / 2, centerPoint[1] - length / 2, 0.};
	std::array<double, 3> upperRight = { centerPoint[0] + height / 2, centerPoint[1] + length / 2, 0. };
	std::array<double, 3> lowerRight = { centerPoint[0] - height / 2, centerPoint[1] + length / 2, 0. };
	std::array<double, 3> lowerLeft = { centerPoint[0] - height / 2, centerPoint[1] - length / 2, 0. };

	return std::array<std::array<double, 3>, 4>({ upperLeft, upperRight, lowerRight, lowerLeft});
}

void scale(std::array<double, 3>& point, double factor)
{
	point[0] *= factor;
	point[1] *= factor;
}


/// Rotates point in place counterclockwise
void rotate(std::array<double, 3>& point, double angleRad)
{
	auto point0 = point[1] * sin(angleRad) + point[0] * cos(angleRad); // y
	auto point1 = point[1] * cos(angleRad) - point[0] * sin(angleRad); // x

	point[0] = point0;
	point[1] = point1;
}

void shift(std::array<double, 3>& point, double shiftX, double shiftY)
{
	point[0] += shiftY;
	point[1] += shiftX;
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

void drawArbitraryRect(cv::Mat& img, std::array<std::array<double, 3>, 4> rect, cv::Scalar color, int thickness = 1)
{
	for (uint32_t i = 0; i < 3; i++)
	{
		cv::line(img, cv::Point(rect[i][1], rect[i][0]), cv::Point(rect[i + 1][1], rect[i + 1][0]), color, thickness = 1);
	}
	cv::line(img, cv::Point(rect[3][1], rect[3][0]), cv::Point(rect[0][1], rect[0][0]), color, thickness);
}

/// Visualizes scene in the coordinates of the robot
void visualizeScene(std::vector<std::array<std::array<double, 3>, 4>> lanesPos, std::array<std::array<double, 3>, 4> objectPos)
{
	auto img = cv::Mat(300, 300, CV_8UC3);

	for (auto& lane : lanesPos)
	{
		for (auto& point : lane)
		{
			scale(point, 10);
		}
		drawArbitraryRect(img, lane, cv::Scalar(0, 0, 255), 2);
	}

	// Draw object scaled
	for (auto& point : objectPos)
	{
		scale(point, 10);
	}
	
	drawArbitraryRect(img, objectPos, cv::Scalar(255, 0, 0), 2);

	// flip since Y axis in the images is inverted
	cv::flip(img, img, 0);

	cv::imshow("demo", img);
	cv::waitKey(0);
}

std::vector<std::array<std::array<double, 3>, 4>> lanesCoordinates(std::array<double, 3> aPos, std::array<double, 3> bPos, double laneWidth, uint32_t nLanes)
{
	double roadYawRad = atan((aPos[0] - aPos[0]) / (bPos[1] - aPos[1]));

	double shiftY = aPos[0];
	double shiftX = aPos[1];

	shift(aPos, shiftX, shiftY);
	shift(bPos, shiftX, shiftY);

	rotate(aPos, -roadYawRad);
	rotate(bPos, -roadYawRad);

	auto lanesCoordinates = std::vector<std::array<std::array<double, 3>, 4>>();

	uint32_t i;
	double y;
	for (i = 0, y = -(nLanes * laneWidth / 2); i < nLanes; i++, y += laneWidth)
	{
		auto laneCoords = std::array<std::array<double, 3>, 4 >({
			std::array<double, 3>({y, 0., 0.}),                  // lower left
			std::array<double, 3>({y, bPos[1], 0.}),             // lower right
			std::array<double, 3>({y + laneWidth, bPos[1], 0.}), // upper right
			std::array<double, 3>({y + laneWidth, 0., 0.})       // upper left
			});

		for (auto& laneCoord : laneCoords)
		{
			shift(laneCoord, shiftX, shiftY);
			rotate(laneCoord, roadYawRad);
		}
		lanesCoordinates.push_back(laneCoords);
	}

	return lanesCoordinates;
}

std::vector<int> whichLanesBusy(std::vector<std::array<std::array<double, 3>, 4>> lanesPoints, std::array<std::array<double, 3>, 4> objPoints)
{
	// transform lanes to make their rectangles regular, with angle = 0
	auto referencePoint = lanesPoints[0][0]; //the lower left of rightest lane

	double roadYawRad = atan((lanesPoints[0][0][0] - lanesPoints[0][1][0])
		/ (lanesPoints[0][1][0] - lanesPoints[0][2][0]));

	double shiftY = referencePoint[0];
	double shiftX = referencePoint[1];

	for (auto& lane : lanesPoints)
	{
		for (auto& point : lane)
		{
			shift(point, -shiftX, -shiftY);
			rotate(point, -roadYawRad);
		}
	}

	for (auto& point : objPoints)
	{
		shift(point, -shiftX, -shiftY);
		rotate(point, -roadYawRad);
	}

	// Check object points in lanes
	auto busyLanes = std::vector<int>();

	for (int i  = 0; i < lanesPoints.size(); i++)
	{
		for (auto& objPoint : objPoints)
		{
			auto lowerLeft = lanesPoints[i][0];
			auto upperRight = lanesPoints[i][2];

			if (objPoint[1] >= lowerLeft[1] && objPoint[1] <= upperRight[1] 
				&& objPoint[0] > lowerLeft[0] && objPoint[0] <= upperRight[0])
			{
				busyLanes.push_back(i + 1);
				break;
			}
		}
	}

	return busyLanes;
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
		shift(objPoint, objPos[1], objPos[0]);
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
	return whichLanesBusy(lanePoints, objPoints);
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

	if(result == std::vector<int>({1}))
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

	if (result == std::vector<int>())
		return true;
	else
		return false;
}

bool test3()
{
	auto rPos = std::array<double, 3>({ 59.96769, 30.30985, 0. }); // lat, long, height
	double rAzimuth = 30;

	auto aPos = std::array<double, 3>({ 59.967681024726275, 30.310046939745639, 0. });
	auto bPos = std::array<double, 3>({ 59.967752830076989, 30.310118554680990, 0. });
	uint32_t nLanes = 4;
	double laneWidth = 2.;

	auto objPos = std::array<double, 3>({ 7., 9., 0. }); // y, x, z
	double objYaw = 0.3235983;
	double objLength = 4.;
	double objWidth = 2.;

	auto result = busyLanes(rPos, rAzimuth, aPos, bPos, nLanes, laneWidth, objPos, objYaw, objLength, objWidth);

	if (result == std::vector<int>({ 1, 2 }))
		return true;
	else
		return false;
}

int main()
{
	//assert(test());
	//assert(test2());
	assert(test3());
	return 0;
}