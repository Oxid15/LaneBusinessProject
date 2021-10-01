#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "LineIntersection.hpp"


double deg2rad(double deg);

double dist(Point a, Point b);

/// Returns 4 box points given center and width with height
std::array<Point, 4> getBoxPoints(const Point& centerPoint, double length, double height);

/// Wrapper around similar function from namespace wgs84 which is 2DoF
Point toCartesian(Point& referencePoint, Point& targetPoint);

std::array<std::array<Point, 2>, 4> getLinesOfRect(std::array<Point, 4> rect);

void drawArbitraryRect(cv::Mat& img, std::array<Point, 4> rect, cv::Scalar color, int thickness = 1);

/// Visualizes scene in the coordinates of the robot
void visualizeScene(std::vector<std::array<Point, 4>> lanesPos, std::array<Point, 4> objectPos,
	uint32_t sceneSizeX = 300,
	uint32_t sceneSizeY = 300);

std::vector<std::array<Point, 4>> lanesCoordinates(Point aPos, Point bPos, double laneWidth, uint32_t nLanes);

bool areRectsIntersect(std::array<Point, 4> rect1, std::array<Point, 4> rect2);

std::vector<uint32_t> whichLanesBusy(std::vector<std::array<Point, 4>> lanesPoints, std::array<Point, 4> objPoints);

std::vector<uint32_t> busyLanes(Point rPos, double rAzimuth,
	Point aPos, Point bPos, uint32_t nLanes, double laneWidth,
	Point objPos, double objYaw, double objLength, double objWidth, bool viz = true);
