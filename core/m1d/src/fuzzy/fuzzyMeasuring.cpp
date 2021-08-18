#include "fuzzyMeasuring.hpp"
#include "mutils.hpp"

using namespace cv;

/**
 * @brief Scores all edges based on the fuzzy functions of the measure handle.
 * @param measureHandle The MeasureHandle object.
 * @param amplitudes Contains edge amplitudes of all valid edges.
 * @param coords Contains relative edge coordinates of all valid edges. Coordinates is the distance
 *  from the left side of the profile line.
 * @param fuzzyThresh Minimum fuzzy score.
 * @return The score and index of each edge that satisfies the minimum fuzzy score.
 */
std::pair<std::vector<double>, std::vector<size_t>>
getScoresPos(const MeasureHandle &measureHandle, const std::vector<double> &amplitudes,
             const std::vector<double> &coords, double fuzzyThresh) {

    double profileLength = measureHandle.getProfileLength();
    size_t numberOfBins = measureHandle.numberOfBins();
    size_t numEdges = coords.size();

    std::vector<double> distances = coords;
    double scaleFactor = profileLength / (numberOfBins - 1);

    if (numberOfBins >= 2)
        for (size_t i = 0; i < numEdges; i++)
            distances[i] *= scaleFactor;

    std::vector<double> scores{};  // Vector to store score for each edge
    std::vector<size_t> returnIdx{};  // Return the indices of edges that satisfy minimum fuzzy score

    size_t numFuncs = measureHandle.getFuzzySet().size();

    // Iterate through all edges
    for (size_t i = 0; i < numEdges; i++) {
        double score = 1.0;

        for (const auto &it : measureHandle.getFuzzySet()) {
            double xVal = 0;

            // Determine the x value for the fuzzy function
            switch (it.first) {
                case FuzzyType::CONTRAST:
                    xVal = abs(amplitudes[i]);
                    break;

                case FuzzyType::POSITION:
                    xVal = distances[i];
                    break;

                case FuzzyType::POSITION_FIRST_EDGE:
                    xVal = distances[i] - distances.front();
                    break;

                case FuzzyType::POSITION_LAST_EDGE:
                    xVal = distances[i] - distances.back();
                    break;

                case FuzzyType::POSITION_CENTER:
                    xVal = distances[i] - profileLength / 2.0;
                    break;

                case FuzzyType::POSITION_END:
                    xVal = distances[i] - profileLength;

                default:
                    break;
            }

            score *= it.second.interpolate(xVal);
        }
        // Take the geometric mean
        if (numFuncs > 0) {
            double meanScore = pow(score, 1.0 / double(numFuncs));

            if (meanScore >= fuzzyThresh) {
                scores.push_back(meanScore);
                returnIdx.push_back(i);
            }
        }
    }

    return std::make_pair(scores, returnIdx);
}

template<typename T>
FuzzyMeasurePosResult
fuzzyMeasurePos(const Mat &img, const T &measureHandle, double sigma, double ampThresh,
                double fuzzyThresh, TransitionType transition) {
    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);
    Mat profile = measureProjection(img, measureHandle);

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

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                  double ampThresh, double fuzzyThresh, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
                            double fuzzyThresh, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArcTransposed>(const cv::Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double ampThresh,
                                     double fuzzyThresh, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                  double ampThresh, double fuzzyThresh, double cx, double cy, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
                            double fuzzyThresh, double cx, double cy, TransitionType transition);

template FuzzyMeasurePosResult
fuzzyMeasurePos<MeasureArcTransposed>(const cv::Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double ampThresh,
                                      double fuzzyThresh, double cx, double cy, TransitionType transition);