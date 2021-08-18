/**
 * @file ransacError.hpp
 * @brief Functions to determine inliers given the shape.
 */

#ifndef MEASURING_RANSACERROR_HPP
#define MEASURING_RANSACERROR_HPP

#include "ransacFit.hpp"

typedef std::vector<std::vector<cv::Point2d>> RansacPoints;

/**
 * @brief Find inliers given a circle and test points.
 * @param shape The circle shape.
 * @param testPoints Points to test for inliers.
 * @param threshold Distance threshold for an inlier.
 * @return The indices of points within testPoints that are inliers.
 */
std::vector<int> arcInliers(const CircleParams &shape, const RansacPoints &testPoints, double threshold);

/**
 * @brief Find inliers given a ellipse and test points.
 * @param shape The ellipse shape.
 * @param testPoints Points to test for inliers.
 * @param threshold Distance threshold for an inlier.
 * @return The indices of points within testPoints that are inliers.
 */
std::vector<int> ellipseInliers(const EllipseParams &shape, const RansacPoints &testPoints, double threshold);

/**
 * @brief Find inliers given a line and test points.
 * @param shape The line shape.
 * @param testPoints Points to test for inliers.
 * @param threshold Distance threshold for an inlier.
 * @return The indices of points within testPoints that are inliers.
 */
std::vector<int> lineInliers(const LineEqParams &shape, const RansacPoints &testPoints, double threshold);

// There is no rectInliers because rectangles, internally, just perform inlier testing on the four lines which represent its sides.

#endif  //MEASURING_RANSACERROR_HPP
