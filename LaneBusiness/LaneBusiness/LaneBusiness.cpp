#include "LaneBusiness.hpp"

const double M_PI = 3.141592653589793;

double deg2rad(double deg) {
	return deg * M_PI / 180.0;
}

double dist(const Point& a, const Point& b)
{
	return sqrt((a.x + b.x) * (a.x + b.x) + (a.y + b.y) * (a.y + b.y));
}

std::array<Point, 4> getBoxPoints(const Point& centerPoint, double length, double height)
{
	Point upperLeft = { centerPoint.y + height / 2, centerPoint.x - length / 2 };
	Point upperRight = { centerPoint.y + height / 2, centerPoint.x + length / 2 };
	Point lowerRight = { centerPoint.y - height / 2, centerPoint.x + length / 2 };
	Point lowerLeft = { centerPoint.y - height / 2, centerPoint.x - length / 2 };

	return std::array<Point, 4>({ upperLeft, upperRight, lowerRight, lowerLeft });
}

/// Wrapper around similar function from namespace wgs84 which is 2DoF
Point toCartesian(const Point& referencePoint, const Point& targetPoint)
{
	std::array<double, 2> refPoint2DoF;
	std::array<double, 2> targetPoint2DoF;
	refPoint2DoF[0] = referencePoint.x;
	refPoint2DoF[1] = referencePoint.y;
	targetPoint2DoF[0] = targetPoint.x;
	targetPoint2DoF[1] = targetPoint.y;

	std::array<double, 2> result2Dof = wgs84::toCartesian(refPoint2DoF, targetPoint2DoF);

	Point result = { result2Dof[0], result2Dof[1] };

	return result;
}

std::array<std::array<Point, 2>, 4> getLinesOfRect(std::array<Point, 4> rect)
{
	auto lines = std::array<std::array<Point, 2>, 4>();
	for (uint32_t i = 0; i < 3; i++)
	{
		lines[i] = std::array<Point, 2>({ rect[i], rect[i + 1] });
	}
	lines[3] = std::array<Point, 2>({ rect[3], rect[0] });

	return lines;
}

void drawArbitraryRect(cv::Mat& img, std::array<Point, 4> rect, cv::Scalar color, int thickness)
{
	auto lines = getLinesOfRect(rect);
	for (auto line : lines)
	{
		cv::line(img, cv::Point(line[0].x, line[0].y), cv::Point(line[1].x, line[1].y), color, thickness = 1);
	}
}

/// Visualizes scene in the coordinates of the robot
void visualizeScene(std::vector<std::array<Point, 4>> lanesPos,
	std::array<Point, 4> objectPos,
	uint32_t sceneSizeX,
	uint32_t sceneSizeY)
{
	auto img = cv::Mat(sceneSizeX, sceneSizeY, CV_8UC3);

	for (auto& lane : lanesPos)
	{
		for (auto& point : lane)
		{
			point *= 10;
			point.x += sceneSizeX / 2;
			point.y += sceneSizeY / 2;
		}
		drawArbitraryRect(img, lane, cv::Scalar(0, 0, 255), 2);
	}

	// Draw object scaled
	for (auto& point : objectPos)
	{
		point *= 10;
		point.x += sceneSizeX / 2;
		point.y += sceneSizeY / 2;
	}

	drawArbitraryRect(img, objectPos, cv::Scalar(255, 0, 0), 2);

	// flip since Y axis in the images is inverted
	cv::flip(img, img, 0);

	cv::imshow("demo", img);
	cv::waitKey(0);
}

std::vector<std::array<Point, 4>> lanesCoordinates(const Point& aPos, const Point& bPos, double laneWidth, uint32_t nLanes)
{
	double roadYawRad = atan((bPos.y - aPos.y) / (bPos.x - aPos.x));

	double shiftY = aPos.y;
	double shiftX = aPos.x;

	Point initbPos = { dist(aPos, bPos), 0. };

	auto lanesCoordinates = std::vector<std::array<Point, 4>>();

	uint32_t i;
	double y;
	for (i = 0, y = -(nLanes * laneWidth / 2); i < nLanes; i++, y += laneWidth)
	{
		auto laneCoords = std::array<Point, 4 >({
			Point({0., y}),                  // lower left
			Point({initbPos.x, y}),              // lower right
			Point({initbPos.x, y + laneWidth}),  // upper right
			Point({0., y + laneWidth})       // upper left
			});

		for (auto& laneCoord : laneCoords)
		{
			laneCoord.x += shiftX;
			laneCoord.y += shiftY;
			laneCoord.rotate(roadYawRad);
		}
		lanesCoordinates.push_back(laneCoords);
	}

	return lanesCoordinates;
}

bool areRectsIntersect(std::array<Point, 4> rect1, std::array<Point, 4> rect2);

std::vector<uint32_t> whichLanesBusy(std::vector<std::array<Point, 4>> lanesPoints, std::array<Point, 4> objPoints)
{
	// transform lanes to make their rectangles regular, with angle = 0
	auto referencePoint = lanesPoints[0][0]; //the lower left of rightest lane

	double roadYawRad = atan((lanesPoints[0][0].y - lanesPoints[0][1].y)
		/ (lanesPoints[0][1].y - lanesPoints[0][2].y));

	double shiftX = referencePoint.x;
	double shiftY = referencePoint.y;

	for (auto& lane : lanesPoints)
	{
		for (auto& point : lane)
		{
			point.x -= shiftX;
			point.y -= shiftY;

			point.rotate(-roadYawRad);
		}
	}

	for (auto& point : objPoints)
	{
		point.x -= shiftX;
		point.y -= shiftY;
		point.rotate(-roadYawRad);
	}

	// Check object in lanes
	auto busyLanes = std::vector<uint32_t>();

	for (uint32_t i = 0; i < lanesPoints.size(); i++)
	{
		if (areRectsIntersect(lanesPoints[i], objPoints))
		{
			busyLanes.push_back(i + 1);
		}
	}
	return busyLanes;
}

std::vector<uint32_t> busyLanes(const Point& rPos, double rAzimuth,
	const Point& aPos, const Point& bPos, uint32_t nLanes, double laneWidth,
	const Point& objPos, double objYaw, double objLength, double objWidth, bool viz)
{
	// Find all the points of an object 
	auto objPoints = getBoxPoints(Point({ 0., 0. }), objWidth, objLength);
	for (auto &objPoint : objPoints)
	{
		objPoint.rotate(-objYaw);
		objPoint.x += objPos.x;
		objPoint.y += objPos.y;
	}

	// Find the road points in cartesian coords
	Point aPosCart = toCartesian(rPos, aPos);
	Point bPosCart = toCartesian(rPos, bPos);

	// Rotate them to match robot's coordinate system
	aPosCart.rotate(deg2rad(-rAzimuth));
	bPosCart.rotate(deg2rad(-rAzimuth));

	// Find all the points of the lanes
	auto lanePoints = lanesCoordinates(aPosCart, bPosCart, laneWidth, nLanes);

	// Visualize the scene in the coordinates of a robot
	if (viz)
	{
		visualizeScene(lanePoints, objPoints);
	}

	// Check where the object is
	return whichLanesBusy(lanePoints, objPoints);
}

bool areRectsIntersect(std::array<Point, 4> rect1, std::array<Point, 4> rect2)
{
	auto lines1 = getLinesOfRect(rect1);
	auto lines2 = getLinesOfRect(rect2);

	for (auto line1 : lines1)
	{
		for (auto line2 : lines2)
		{
			if (areLinesIntersect(line1, line2))
			{
				return true;
			}
		}
	}

	return false;
}
