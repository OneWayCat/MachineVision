/**
 * @file measuring.hpp
 * @brief Functions for 1D measuring.
 */

#ifndef MEASURING_MEASURING_HPP
#define MEASURING_MEASURING_HPP

#include "measureHandle.hpp"


cv::Mat measureProjection(const cv::Mat &img, const MeasureHandle &measureHandle, double cx, double cy);





#endif