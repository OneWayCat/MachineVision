

#ifndef MEASURING_POSBEAD_HPP
#define MEASURING_POSBEAD_HPP

#include "beadModel.hpp"
#include "mathutils.hpp"

//TODO: Doc

// 1 / Golden
#define INVPHI (sqrt(5) - 1) / 2.0
// 1 / Golden^2
#define INVPHI2 (3 - sqrt(5)) / 2.0

double distPtToSegSq(const cv::Point2i &x, const cv::Point2i &p1, const cv::Point2i &p2);

//double distPtToContourSq(const cv::Point2i &x, const Contour &cont);

std::pair<int, int> gsSearch(const cv::Point2i &currentPt, const Contour &denseSpline, int start, int end);

#endif
