/**
 * @file mathutils.hpp
 * @brief Geometric and mathematical conversion/calculation utility functions.
 */

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <random>

/**
 * Converts an angle in degrees to radians.
 *
 * @param degrees Angle in degrees.
 * @return Angle converted to radians.
 */
double deg2rad(double degrees);

/**
 * Converts an angle in radians to degrees.
 *
 * @param radians angle in radians.
 * @return Angle converted to degrees.
 */
double rad2deg(double radians);

// Map an angle to [-pi, pi]
double mapToPi(double angle);

// Map an angle to [0, 2pi]
double mapTo2Pi(double angle);

// Get the square of an integer.
int square(int x);

/**
 * Calculates the euclidean distance between the points (x1, y1) and (x2, y2)
 *
 * @param x1
 * @param x2
 * @param y1
 * @param y2
 * @return Euclidean distance.
 */
double euclideanDistance(double x1, double x2, double y1, double y2, bool root = true);

// Calculate the euclidean distance between two points.
double euclideanDistance(const cv::Point &pt1, const cv::Point &pt2, bool root = true);

// Calculate the manhattan distance between two points.
int manhattanDist(const cv::Point2i &p1, const cv::Point2i &p2);

// Rotate a vector (x, y) by an angle, counterclockwise.
void rotate(double &x, double &y, double sinPhi, double cosPhi);

// Generate a random number in a certain range. Result will be rounded down for integers.
template<typename T>
T randomRange(T lower, T upper) {
	std::random_device r;
	std::default_random_engine rand(r());
	std::uniform_real_distribution<double> unif(lower, upper);

	return (T) unif(rand);
}

//TODO:Doc
cv::Point2d getPerpOffset(const cv::Point2d &start, const cv::Point2d &end, double distance);

cv::Point2d getPerpOffset(const cv::Point2d &start, const cv::Point2d &end, double position, double offset);

// Find the squared magnitude of a vector.
double magnitudeSq(const cv::Point &v);

#endif
