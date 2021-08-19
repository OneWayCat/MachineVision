#include "fuzzyMeasuring.hpp"
#include "mutils.hpp"

using namespace cv;



//template<typename T>
//FuzzyMeasurePosResult
//fuzzyMeasurePos(const Mat &img, const T &measureHandle, double sigma, double ampThresh,
//                double fuzzyThresh, TransitionType transition)

template<typename T>
FuzzyMeasurePosResult
fuzzyMeasurePos(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                double fuzzyThresh, double cx, double cy, TransitionType transition) {
    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);
    Mat profile = measureProjection(img, measureHandle, cx, cy);

    // Get edge coordinates and amplitudes
    auto returnVal = getEdgeAmplitudes(profile, measureHandle, sigma, ampThresh, transition);
    std::vector<double> &coords = returnVal.first;
    std::vector<double> &amplitudes = returnVal.second;

    // Get fuzzy scores
    auto scoreReturnVal = getScoresPos(measureHandle, amplitudes, coords, fuzzyThresh);
    std::vector<double> &scores = scoreReturnVal.first;
    std::vector<size_t> &validIdx = scoreReturnVal.second;

    // Only take the portion of edge coords and amplitudes that satisfy minimum fuzzy score
    std::vector<double> validCoords{};
    std::vector<double> validAmps{};

    for (const size_t &i : validIdx) {
        validCoords.push_back(coords[i]);
        validAmps.push_back(amplitudes[i]);
    }

    // Find row, column coordinates and distance between edges
    PosDistanceResult pdResult = findEdgePos(measureHandle, validCoords);

    return FuzzyMeasurePosResult(pdResult, validAmps, scores);
}

//template FuzzyMeasurePosResult
//fuzzyMeasurePos<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
//                                  double ampThresh, double fuzzyThresh, TransitionType transition);
//
//template FuzzyMeasurePosResult
//fuzzyMeasurePos<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
//                            double fuzzyThresh, TransitionType transition);
//
//template FuzzyMeasurePosResult
//fuzzyMeasurePos<MeasureArcTransposed>(const cv::Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double ampThresh,
//                                     double fuzzyThresh, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                  double ampThresh, double fuzzyThresh, double cx, double cy, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
                            double fuzzyThresh, double cx, double cy, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArcTransposed>(const cv::Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double ampThresh,
                                      double fuzzyThresh, double cx, double cy, TransitionType transition);