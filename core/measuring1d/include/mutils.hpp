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

/**
 * @brief Perform non-max suppression of values with the index immediately before and immediately after.
 * @param profile 1D array containing edge amplitudes to be suppressed
 * @return The suppressed profile.
 */
cv::Mat nonMaxSuppress(cv::Mat &profile);

// Validate image, sigma, threshold.
void validateArgs(const cv::Mat &img, double sigma, double threshold);

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

/**
 * @brief Starting at index, finds the index of the first element with the given sign. Helper function for measurePairs.
 * @details If findStrongest == true, finds the index of the element with the given sign and the maximum amplitude,
 *  but only searches until there is an element with the opposite sign.
 *
 * @param arr Input array.
 * @param sign Sign bit to look for. 0 if positive, 1 if negative.
 * @param index Starting index.
 * @param findStrongest Whether to search for the element with the maximum amplitude or not.
 * @return Index of the first element with the given sign. -1 if not found.
 */
int nextSignedElement(const std::vector<double> &arr, bool sign, int index, bool findStrongest);

#endif //MEASURING_MUTILS_HPP
