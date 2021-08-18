#include "beadModel.hpp"

// Fit a contour to a NURBS spline.
tinynurbs::RationalCurve<double> fitSplineContour(const Contour& ctrlPts);

void drawSplineContour(cv::Mat& img, const tinynurbs::RationalCurve<double>& crv,
	unsigned int numPts, const cv::Scalar& color);
