#include "beadMeasuring.hpp"

using namespace cv;

FuzzyMeasurePairsResult FuzzyMeasuringFunctor::operator()(const Mat& img, const MeasureRectangle& rect) const {
	return fuzzyMeasurePairs(img, rect, sigma, ampThresh, fuzzyThresh, transition);
}

MeasureRectangle MeasureRectTemplate::operator()(int row, int col, double phi) const {
	MeasureRectangle retRect(halfWidth, halfHeight, row, col, phi);

	for (auto& set : fuzzySet)
		retRect.setFuzzyFunc(set.second, set.first);

	return retRect;
}

void MeasureRectTemplate::addFuzzy(const FuzzyFunc& func, const FuzzyType& type) {
	if (type == FuzzyType::SIZE_DIFF or type == FuzzyType::SIZE_ABS_DIFF)
		throw std::invalid_argument("Fuzzy type not available for bead inspection");

	fuzzySet.insert({ type, func });
}