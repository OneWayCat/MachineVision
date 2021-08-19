/**
 * @file fuzzyMeasuring.hpp
 * @brief Functions for 1D fuzzy measuring.
 */

#ifndef MEASURING_FUZZYMEASURING_HPP
#define MEASURING_FUZZYMEASURING_HPP

#include "measuring.hpp"





template<typename T>
FuzzyMeasurePosResult
fuzzyMeasurePos(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                double fuzzyThresh, double cx, double cy, TransitionType transition = TransitionType::ALL);






#endif //MEASURING_FUZZYMEASURING_HPP
