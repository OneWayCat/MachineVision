#include "posBead.hpp"


using namespace cv;

/** Calculate the squared distance between a point to a line segment.
 * @param x Point to test.
 * @param p1 Beginning point of the line segment.
 * @param p2 Ending point of the line segment.
 */
double distPtToSegSq(const Point2i &x, const Point2i &p1, const Point2i &p2) {
	const Point2i lineVec = p2 - p1;
	const Point2i x2p1 = x - p1;

	double r = lineVec.dot(x2p1);
	r /= magnitudeSq(lineVec);

	if (r < 0)
		return magnitudeSq(x2p1);
	else if (r > 1)
		return magnitudeSq(p2 - x);

	return magnitudeSq(x2p1) - r * r * magnitudeSq(lineVec);
}

//// Calculate the squared distance between a point to a contour.
//double distPtToContourSq(const Point2i &x, const Contour &cont) {
//	// Find the distance between each segment of the contour and the test point and return the minimum.
//	// Since cont will always be the beadContour field of a BeadModel,
//	// It will always have at least two points.
//	std::vector<double> dists;
//	dists.reserve(cont.size());
//
//	for (size_t i = 1; i < cont.size(); i++)
//		dists.push_back(distPtToSegSq(x, cont[i - 1], cont[i]));
//
//	return *std::min_element(dists.begin(), dists.end());
//}

std::pair<int, int>
gsSearch(const Point2i &currentPt, const Contour &denseSpline, int start, int end) {
	double constexpr tolerance = 1e-5;

	int minInd = min(start, end);
	int maxInd = max(start, end);
	int deltaInd = maxInd - minInd;

	if (deltaInd <= tolerance) {
		return std::make_pair(minInd, maxInd);
	}

	int n = int(std::ceil(std::log(tolerance / deltaInd) / std::log(INVPHI)));

	double interval1 = minInd + INVPHI2 * deltaInd;
	double interval2 = minInd + INVPHI * deltaInd;
	double dist1 = euclideanDistance(currentPt, denseSpline[int(interval1)]);
	double dist2 = euclideanDistance(currentPt, denseSpline[int(interval2)]);

	for (int i = 0; i < n - 1; i++) {

		if (dist1 < dist2) {
			maxInd = int(interval2);
			interval2 = interval1;
			dist2 = dist1;
			deltaInd = INVPHI * deltaInd;
			interval1 = minInd + INVPHI2 * deltaInd;
			dist1 = euclideanDistance(currentPt, denseSpline[int(interval1)]);

		} else {
			minInd = int(interval1);
			interval1 = interval2;
			dist1 = dist2;
			deltaInd = INVPHI * deltaInd;
			interval2 = minInd + INVPHI * deltaInd;
			dist2 = euclideanDistance(currentPt, denseSpline[int(interval2)]);
		}

		if (deltaInd == 0) {
			if (dist1 < dist2)
				return std::make_pair(minInd, interval2);
			else
				return std::make_pair(interval1, maxInd);
		}
	}

	if (dist1 < dist2)
		return std::make_pair(minInd, interval2);
	else
		return std::make_pair(interval1, maxInd);
}