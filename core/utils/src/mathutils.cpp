#include "mathutils.hpp"

#include <boost/math/constants/constants.hpp>

using namespace boost::math;
using namespace cv;

double deg2rad(double degrees) {
	return degrees * double_constants::degree;
}

double rad2deg(double radians) {
	return radians * double_constants::radian;
}

double mapToPi(double angle) {
	double result = mapTo2Pi(angle);

	if (result > double_constants::pi)
		result -= double_constants::two_pi;

	return result;
}

double mapTo2Pi(double angle) {
	double twoPi = double_constants::two_pi;

	if (angle > twoPi)
		return fmod(angle, twoPi);

	if (angle < 0)
		return fmod(angle, twoPi) + twoPi;

	return angle;
}

int square(int x) {
	return x * x;
}

double euclideanDistance(double x1, double x2, double y1, double y2, bool root) {
	double dX = x1 - x2;
	double dY = y1 - y2;

	if (root)
		return sqrt(dX * dX + dY * dY);
	else
		return dX * dX + dY * dY;
}

double euclideanDistance(const Point &p1, const Point &p2, bool root) {
	double dX = double(p1.x) - p2.x;
	double dY = double(p1.y) - p2.y;

	if (root)
		return sqrt(dX * dX + dY * dY);
	else
		return dX * dX + dY * dY;
}

int manhattanDist(const Point2i &p1, const Point2i &p2) {
	return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

void rotate(double &x, double &y, double sinPhi, double cosPhi) {
	double tempX = x;
	double tempY = y;

	x = tempX * cosPhi - tempY * sinPhi;
	y = tempX * sinPhi + tempY * cosPhi;
}

Point2d getPerpOffset(const Point2d &start, const Point2d &end, double distance) {
	Point2d M = (start + end) / 2;
	Point2d p = start - end;
	Point2d n = Point2d(-p.y, p.x);
	int norm_length = (int) sqrt((n.x * n.x) + (n.y * n.y));
	n.x /= norm_length;
	n.y /= norm_length;
	return (M + (distance * n));
}

Point2d getPerpOffset(const Point2d &start, const Point2d &end, double position, double offset) {
	// p3 is located at the x or y delta * position + p1x or p1y original.
	const double dx = end.x - start.x;
	const double dy = end.y - start.y;

	const Point2d p3{dx * position + start.x, dy * position + start.y};

	// Add 90 degrees to get perpendicular offset
	const double perpAngle = atan2(dy, dx) + double_constants::half_pi;

	// Return the point at given angle and distance from p3.
	return Point2d{p3.x + cos(perpAngle) * offset, p3.y + sin(perpAngle) * offset};
}

double magnitudeSq(const Point &v) {
	return square(v.x) + square(v.y);
}