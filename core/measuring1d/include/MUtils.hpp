
#ifndef MEASURING_MUTILS_HPP
#define MEASURING_MUTILS_HPP

#include "opencv2/core.hpp"

/**
 * @brief Perform non-max suppression of values with the index immediately before and immediately after.
 * @param profile 1D array containing edge amplitudes to be suppressed
 * @return The suppressed profile.
 */
cv::Mat nonMaxSuppress(cv::Mat &profile);

// Validate image, sigma, threshold.
void validateArgs(const cv::Mat &img, double sigma, double threshold);

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

#endif
