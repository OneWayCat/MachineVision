#include "fitSpline.hpp"

using namespace cv;

const Point2i DUMMY_PT{-1, -1};

// Pick the pair with the highest score out of all edge pairs
std::pair<Point2i, Point2i> pickBestPair(const FuzzyMeasurePairsResult &result) {
	if (result.fuzzyScores.empty())
		return std::make_pair(DUMMY_PT, DUMMY_PT);

	// Find the index of the pair with the highest score
	const std::vector<double> &scores = result.fuzzyScores;
	const auto maxScorePtr = std::max_element(scores.begin(), scores.end());
	const auto maxIdx = maxScorePtr - scores.begin();

	return std::make_pair(result.posFirst[maxIdx], result.posSecond[maxIdx]);
}

std::vector<Point2i> BeadModel::getAnchorPts(const Mat &img, int index) {
	// If the anchor point is already calculated, just return it
	if (anchorPts[index][0] != DUMMY_PT)
		return anchorPts[index];

	const glm::dvec3 tangent = tinynurbs::curveTangent(spline, index / (double(numMeasures) - 1));
	const double tangentAngle = atan2(tangent[1], tangent[0]);

	const Point2i &center = measureCenters[index];
	const MeasureRectangle rect = handleFunctor(center.x, center.y, tangentAngle);

	const FuzzyMeasurePairsResult measureResult = fuzzyFunctor(img, rect);

	const auto anchorPoints = pickBestPair(measureResult);

	std::vector<Point2i> ret = {anchorPoints.first, anchorPoints.second};
	anchorPts[index] = ret;

	return ret;
}

BeadInspectionResult BeadModel::applyBeadModel(const Mat &img, const MeasureRectTemplate &rectTemplate,
											   const FuzzyMeasuringFunctor &measureFunctor) {

	CV_Assert(img.depth() == CV_8U and img.channels() == 1);

	// Initialize anchorPts
	if (anchorPts.empty()) {
		anchorPts.reserve(numMeasures);
		for (int i = 0; i < numMeasures; i++) {
			std::vector<Point2i> dummyPts = {DUMMY_PT, DUMMY_PT};
			anchorPts.push_back(dummyPts);
		}
	}

	// Store the functors for dynamic creation later on
	handleFunctor = rectTemplate;
	fuzzyFunctor = measureFunctor;

	// Generate the NURBS spline
	if (spline.control_points.empty()) {
		spline = fitSplineContour(beadContour);
	}

	// Pre sample a dense represenation of the spline. Used for out of position calculations during tracking.
	// TODO: To be used by drawing function(?)
	int NUM_SAMPLES = 20 * numMeasures;
	if (denseSpline.empty()) {
		denseSpline.reserve(NUM_SAMPLES);
		for (int i = 0; i < NUM_SAMPLES; i++) {
			double scaled = (double(i) / NUM_SAMPLES) * (numMeasures - 1);
			const glm::dvec3 pt = tinynurbs::curvePoint(spline, scaled / (double(numMeasures) - 1));
			denseSpline.emplace_back(int(pt[0]), int(pt[1]));
		}
	}

//	Mat temp = img.clone();
//	cvtColor(temp, temp, COLOR_GRAY2BGR);
//	for (Point2i p : denseSpline) {
//		circle(temp, Point2i(p.y, p.x), 3, Scalar(0, 255, 0), -1);
//	}
//	imshow("dense", temp);
//	waitKey(0);

	// Pre sample the centers of the MeasureHandles along the spline
	if (measureCenters.empty()) {
		measureCenters.reserve(numMeasures);
		for (int i = 0; i < numMeasures; i++) {
			const glm::dvec3 pt = tinynurbs::curvePoint(spline, i / (double(numMeasures) - 1));
			measureCenters.emplace_back(int(pt[0]), int(pt[1]));
		}
	}
	return trackBead(img);
}
