#include "fitSpline.hpp"
#include "arrutils.hpp"
#include "mathutils.hpp"

using namespace cv;

std::vector<double> generateKnotVector(const unsigned int degree,
	const Contour& ctrlPts) {
	std::vector<double> dists;
	dists.reserve(ctrlPts.size());

	dists.push_back(0);

	// Calculate cumulative manhattan distances
	for (size_t i = 1; i < ctrlPts.size(); i++) {
		const Point2i& currentPt = ctrlPts[i - 1];
		const Point2i& nextPt = ctrlPts[i];

		const int dist = manhattanDist(currentPt, nextPt);

		if (i == 0)
			dists.push_back(double(dist));
		else
			dists.push_back(double(dist) + dists.back());
	}

	// Normalize distances
	for (double& dist : dists)
		dist = dist / dists.back();

	std::vector<double> knots;
	knots.reserve(ctrlPts.size() + 2 * size_t(degree));

	// Generate knot vector based on distances
	push_n(knots, 0, size_t(degree) + 1);

	for (int i = degree + 1; i < ctrlPts.size(); i++) {
		double average = 0;

		for (int j = i - degree; j < i; j++)
			average += dists[j];

		knots.push_back(average / degree);
	}

	push_n(knots, 1, size_t(degree) + 1);

	return knots;
}

tinynurbs::RationalCurve<double> fitSplineContour(const Contour& ctrlPts) {
	constexpr int WEIGHT = 15;

	const size_t numCtrlPts = ctrlPts.size();

	tinynurbs::RationalCurve<double> crv;
	crv.control_points.reserve(numCtrlPts);

	// Insert control points
	for (const Point2d& pt : ctrlPts)
		crv.control_points.emplace_back(pt.x, pt.y, 0);

	crv.degree = std::min(3, int(numCtrlPts - 1));
	crv.knots = generateKnotVector(crv.degree, ctrlPts);

	crv.weights.reserve(numCtrlPts);
	push_n(crv.weights, WEIGHT, numCtrlPts);

	return crv;
}

void drawSplineContour(Mat& img, const tinynurbs::RationalCurve<double>& crv,
	const unsigned int numPts, const Scalar& color) {

	constexpr int radius = 2;

	for (unsigned int i = 0; i <= numPts; i++) {
		const glm::dvec3 pt = tinynurbs::curvePoint(crv, i / double(numPts));
		const int col = int(pt[1]);
		const int row = int(pt[0]);

		const Point2i cvPt{ col, row };
		circle(img, cvPt, radius, color, FILLED);
	}
}
