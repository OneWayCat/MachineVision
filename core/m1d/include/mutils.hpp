#ifndef MEASURING_MUTILS_HPP
#define MEASURING_MUTILS_HPP

#include "measuring.hpp"

// Helper functions for 1D measuring.

// Helper function for creating and drawing MeasureRectangles.

/**
 * @brief Get the 4 points of a measureRectangle as a vector of Point2d structs before adjusting to center of mRect.
 * @param measureRectangle
 * @return 4 points of the rectangle.
 */
std::vector<cv::Point2d> getRectPoints(const MeasureRectangle &measureRectangle);

// Helper functions for measuring and fuzzy measuring functions



// Get the edge coordinates and amplitudes.
std::pair<std::vector<double>, std::vector<double>>
getEdgeAmplitudes(cv::Mat &profile, const MeasureHandle &measureHandle, double sigma, double threshold,
                  TransitionType transition);

// Find the distance between 2 edges in a measure rectangle.
double findDistance(const MeasureRectangle &measureHandle, double firstCoord, double secondCoord);

// Same function, but for arcs.
double findDistance(const MeasureArc &measureHandle, double firstCoord, double secondCoord);

/**
 * @brief Find the row and column coordinates of the points specified by the coords vector inside the
 * measure handle and the distances between each pair of consecutive points found.
 *
 * @param measureHandle The measureHandle object
 * @param coords A vector that specifies where the points are inside measureHandle.
 * @param select Whether to return all points or just the first or last point.
 * @return A structure representing the position of the points and the distance between each pair of consecutive points.
 */
PosDistanceResult
findEdgePos(const MeasureRectangle &measureHandle, const std::vector<double> &coords, SelectType select = SelectType::ALL);

/**
 * @brief The same method, overloaded for MeasureArc.
 *
 * @param measureHandle The measureHandle object
 * @param coords A vector that specifies where the points are inside measureHandle.
 * @param select Whether to return all points or just the first or last point.
 * @return A structure representing the position of the points and the distance between each pair of consecutive points.
 */
PosDistanceResult
findEdgePos(const MeasureArc &measureHandle, const std::vector<double> &coords, SelectType select = SelectType::ALL);

/**
 * @brief The same method, overloaded for MeasureArc.
 *
 * @param measureHandle The measureHandle object
 * @param coords A vector that specifies where the points are inside measureHandle.
 * @param select Whether to return all points or just the first or last point.
 * @return A structure representing the position of the points and the distance between each pair of consecutive points.
 */
PosDistanceResult
findEdgePos(const MeasureArcTransposed &measureHandle, const std::vector<double> &coords, SelectType select = SelectType::ALL);



#endif //MEASURING_MUTILS_HPP
