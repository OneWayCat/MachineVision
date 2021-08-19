
#ifndef MEASURING_FUZZYMEASURING_HPP
#define MEASURING_FUZZYMEASURING_HPP

#include "FuzzyStructs.hpp"
#include "MUtils.hpp"
#include "EdgeElement.hpp"

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
getScoresPos(const EdgeElement &measureHandle, const std::vector<double> &amplitudes, const std::vector<double> &coords, double fuzzyThresh);

/**
 * Return the score of a single edge pair based on the fuzzy functions.
 */
double getScoreSinglePair(const cv::Mat &profile, const EdgeElement &measureHandle, double firstAmp, double firstDist,
                          double secondAmp, double secondDist, double firstPairCenter, double lastPairCenter,
                          double firstEdgeDist, double lastEdgeDist);

/**
 * @brief Scores all edge pairs based on the fuzzy functions of the measure handle.
 * @param profile The 1D profile returned by measureProjection.
 * @param measureHandle The MeasureHandle object.
 * @param firstAmps
 * @param firstCoords
 * @param secondAmps
 * @param secondCoords
 * @return A 2D array of fuzzy scores, where arr[i][j] is score for the pair with the ith first edge and
 *  jth second edge. The element is -1 if the pair is invalid.
 *
 *  Returns an empty vector if there are no valid edge pairs.
 */
std::vector<double>
getScoresAllPairs(const cv::Mat &profile, const EdgeElement &measureHandle, const std::vector<double> &firstAmps,
                  const std::vector<double> &firstCoords, const std::vector<double> &secondAmps,
                  const std::vector<double> &secondCoords);

/*
 * Find the (rowIndex, colIndex, score) of the elements of the scores matrix that match maxScore.
 */
std::vector<std::tuple<int, int, double>>
findMaxPairs(const std::vector<double> &scores, size_t nRows, size_t nCols, double maxScore);

std::vector<std::tuple<size_t, size_t, double>>
findValidPairs(std::vector<double> &allScores, size_t nRows, size_t nCols, double fuzzyThresh, const std::vector<double> &firstCoords, const std::vector<double> &secondCoords);

/**
 * @fn FuzzyMeasurePosResult fuzzyMeasurePos(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                    double fuzzyThresh, TransitionType transition = TransitionType::ALL);
 *
 * @brief Finds the position of straight edges perpendicular to a rectangle or an annular arc.
 * @details Edge detection functions exactly the same as implemented in measurePos.
    However there is now a scoring system for every edge at the end which scores edges based on
    whatever was specified by the user before calling this method. Edges scored above the threshold
    supplied will be returned.

 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing. \n
 *  Range of values: 0.4 <= sigma <= 100
 * @param ampThresh Minimum edge amplitude. \n
 *  Range of values: 0 <= threshold <= 255
 * @param fuzzyThresh Minimum fuzzy score. \n
 *  Range of values: 0.0 <= fuzzyThresh <= 1.0
 * @param transition Detect all edges, or just dark->light or light->dark edges.
 * @return A FuzzyMeasurePosResult struct containing the row, column coordinate, amplitude of each edge, distance
 *  between each consecutive edge and the fuzzy score of each edge.
 */
template<typename T>
FuzzyMeasurePosResult
fuzzyMeasurePos(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                double fuzzyThresh, TransitionType transition = TransitionType::ALL) {
    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);
    cv::Mat profile = measureProjection(img, measureHandle);

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
    EdgeResults pdResult = findEdgePos(measureHandle, validCoords);

    return {pdResult, validAmps, scores};
}

/**
 * @fn FuzzyMeasurePairsResult fuzzyMeasurePairs(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                  double fuzzyThresh, TransitionType transition = TransitionType::ALL);
 * @brief Find the position of straight edge pairs perpendicular to a rectangle or an annular arc.
 * @details This function is similar to measurePairs, but it uses fuzzy functions specified by the user to filter
 *  and score the edge pairs. The pairs are formed in a way such that they do not overlap or include each other.
 *
 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing. \n
 *  Range of values: 0.4 <= sigma <= 100
 * @param ampThresh Minimum edge amplitude. \n
 *  Range of values: 0 <= threshold <= 255
 * @param fuzzyThresh Minimum fuzzy score. \n
 *  Range of values: 0.0 <= fuzzyThresh <= 1.0
 * @param transition Detect all edges, or just dark->light or light->dark edges.
 * @return A FuzzyMeasurePairsResult struct containing the row, column coordinate, amplitude of each edge pair, the distance
 *  between the edges of each pair, the distance between consecutive edge pairs and the fuzzy score of each edge pair.
 */
template<typename T>
FuzzyMeasurePairsResult
fuzzyMeasurePairs(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                  double fuzzyThresh, TransitionType transition = TransitionType::ALL) {

    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);

    cv::Mat profile = measureProjection(img, measureHandle);

    // We create a copy because we need the original profile to score the pairs for FuzzyType::GRAY
    cv::Mat profileCopy = profile.clone();

    // Get edge coordinates and amplitudes
    auto returnVal = getEdgeAmplitudes(profileCopy, measureHandle, sigma, ampThresh, TransitionType::ALL);
    std::vector<double> &coords = returnVal.first;
    std::vector<double> &amplitudes = returnVal.second;

    // Make sure there are at least 2 edges (for at least one edge pair)
    if (amplitudes.size() < 2)
        return FuzzyMeasurePairsResult{};

    // If transition is ALL, the first edge determines the polarity of edge pairs to detect
    if (transition == TransitionType::ALL)
        transition = amplitudes.front() > 0 ? TransitionType::POSITIVE : TransitionType::NEGATIVE;

    bool sign = transition == TransitionType::NEGATIVE;

    // Find edge pairs
    // "Index" refers to the index of the edge within the coords or amplitudes vector
    std::vector<int> firstIndex{};
    std::vector<int> secondIndex{};

    std::vector<double> firstAmps{};
    std::vector<double> secondAmps{};

    std::vector<double> firstCoords{};
    std::vector<double> secondCoords{};

    // Get the amplitudes and coordinates of the first and second edge of each pair
    for (int i = 0; i < amplitudes.size(); i++) {
        if (signbit(amplitudes[i]) == sign) {
            firstIndex.push_back(i);
            firstAmps.push_back(amplitudes[i]);
            firstCoords.push_back(coords[i]);
        } else {
            secondIndex.push_back(i);
            secondAmps.push_back(amplitudes[i]);
            secondCoords.push_back(coords[i]);
        }
    }

    // No valid pairs
    if (firstIndex.empty() or secondIndex.empty())
        return FuzzyMeasurePairsResult{};

    // Get the 2D array containing fuzzy scores of all possible edge pairs (-1 if invalid)
    auto allScores = getScoresAllPairs(profile, measureHandle, firstAmps, firstCoords, secondAmps, secondCoords);

    size_t nRows = firstIndex.size();
    size_t nCols = secondIndex.size();

    if (allScores.empty())
        return FuzzyMeasurePairsResult{};

    auto validPairs = findValidPairs(allScores, nRows, nCols, fuzzyThresh, firstCoords, secondCoords);
    size_t numValidPairs = validPairs.size();

    // Sort the pairs so that it's in increasing order for first indices
    // std::sort automatically sorts a vector of tuples by its first element
    std::sort(validPairs.begin(), validPairs.end());

    // Find return values
    std::vector<double> finalFirstCoords{};
    std::vector<double> finalSecondCoords{};

    std::vector<double> finalFirstAmps{};
    std::vector<double> finalSecondAmps{};

    std::vector<double> finalScores{};

    for (const auto &pair : validPairs) {
        size_t first = std::get<0>(pair);
        size_t second = std::get<1>(pair);

        finalFirstCoords.push_back(firstCoords[first]);
        finalSecondCoords.push_back(secondCoords[second]);

        finalFirstAmps.push_back(firstAmps[first]);
        finalSecondAmps.push_back(secondAmps[second]);

        finalScores.push_back(std::get<2>(pair));
    }

    // Find row and column coordinates of first edges, second edges, and center points between each pair
    auto firstPosDist = findEdgePos(measureHandle, finalFirstCoords);
    auto &finalFirstPos = firstPosDist.pos;

    auto secondPosDist = findEdgePos(measureHandle, finalSecondCoords);
    auto &finalSecondPos = secondPosDist.pos;

    std::vector<cv::Point2d> finalCenterPos{};

    for (int i = 0; i < numValidPairs; i++)
        finalCenterPos.push_back((finalFirstPos[i] + finalSecondPos[i]) / 2.0);

    // Find intra distance of each pair
    std::vector<double> intraDist{};
    std::vector<double> interDist{};

    for (size_t i = 0; i < numValidPairs; i++)
        intraDist.push_back(findDistance(measureHandle, finalFirstCoords[i], finalSecondCoords[i]));

    // Find inter distance between pairs
    if (numValidPairs >= 2)
        for (size_t i = 0; i < numValidPairs - 1; i++)
            interDist.push_back(findDistance(measureHandle, finalSecondCoords[i], finalFirstCoords[i + 1]));

    return FuzzyMeasurePairsResult{finalFirstPos, finalFirstAmps,
                                   finalSecondPos, finalSecondAmps,
                                   intraDist, interDist,
                                   finalCenterPos, finalScores
    };
}

/**
 * @fn FuzzyMeasurePairingsResult fuzzyMeasurePairings(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                     double fuzzyThresh, TransitionType transition = TransitionType::ALL, unsigned int numPairs = 0);
 * @brief Find the position of straight edge pairs perpendicular to a rectangle or an annular arc.
 * @details This function is similar to fuzzyMeasurePairs, but it is also possible to return overlapping edge pairs or
 *          pairs that include one another. Only the highest scoring numPairs pairs are returned.
 *
 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing. \n
 *  Range of values: 0.4 <= sigma <= 100
 * @param ampThresh Minimum edge amplitude. \n
 *  Range of values: 0 <= threshold <= 255
 * @param fuzzyThresh Minimum fuzzy score. \n
 *  Range of values: 0.0 <= fuzzyThresh <= 1.0
 * @param transition Detect all edges, or just dark->light or light->dark edges.
 * @param numPairs Number of best pairs to return, or 0 to return all pairs above fuzzyThresh.
 * @return A FuzzyMeasurePairingsResult struct containing the row, column coordinate, amplitude of each edge pair, the distance
 *  between the edges of each pair, and the fuzzy score of each edge pair.
 */
template<typename T>
FuzzyMeasurePairingsResult
fuzzyMeasurePairings(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                     double fuzzyThresh, TransitionType transition = TransitionType::ALL, unsigned int numPairs = 0) {

    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);

    cv::Mat profile = measureProjection(img, measureHandle);

    // We create a copy because we need the original profile to score the pairs for FuzzyType::GRAY
    cv::Mat profileCopy = profile.clone();

    // Get edge coordinates and amplitudes
    auto returnVal = getEdgeAmplitudes(profileCopy, measureHandle, sigma, ampThresh, TransitionType::ALL);
    std::vector<double> &coords = returnVal.first;
    std::vector<double> &amplitudes = returnVal.second;

    // Make sure there are at least 2 edges (for at least one edge pair)
    if (amplitudes.size() < 2)
        return FuzzyMeasurePairsResult{};

    // If transition is ALL, the first edge determines the polarity of edge pairs to detect
    if (transition == TransitionType::ALL)
        transition = amplitudes.front() > 0 ? TransitionType::POSITIVE : TransitionType::NEGATIVE;

    bool sign = transition == TransitionType::NEGATIVE;

    // Find edge pairs
    // "Index" refers to the index of the edge within the coords or amplitudes vector
    std::vector<int> firstIndex{};
    std::vector<int> secondIndex{};

    std::vector<double> firstAmps{};
    std::vector<double> secondAmps{};

    std::vector<double> firstCoords{};
    std::vector<double> secondCoords{};

    // Get the amplitudes and coordinates of the first and second edge of each pair
    for (int i = 0; i < amplitudes.size(); i++) {
        if (signbit(amplitudes[i]) == sign) {
            firstIndex.push_back(i);
            firstAmps.push_back(amplitudes[i]);
            firstCoords.push_back(coords[i]);
        } else {
            secondIndex.push_back(i);
            secondAmps.push_back(amplitudes[i]);
            secondCoords.push_back(coords[i]);
        }
    }

    // No valid pairs
    if (firstIndex.empty() or secondIndex.empty())
        return FuzzyMeasurePairsResult{};

    // Get the 2D array containing fuzzy scores of all possible edge pairs (-1 if invalid)
    auto allScores = getScoresAllPairs(profile, measureHandle, firstAmps, firstCoords, secondAmps, secondCoords);

    if (allScores.empty())
        return FuzzyMeasurePairsResult{};

    size_t nCols = secondIndex.size();

    /* Before sorting the scores, we have to keep track of the indices of each element in the 2D array
     * when we sort, so make a 2D array of tuples */

    // The order of the tuple is <score, rowIdx, columnIdx>
    std::vector<std::tuple<double, size_t, size_t>> validPairs(allScores.size());
    size_t count = 0;

    for (const double &score : allScores) {
        validPairs.emplace_back(score, count / nCols, count % nCols);
        count++;
    }

    // Sort the pairs in decreasing order of score
    std::sort(validPairs.begin(), validPairs.end(),
              [](std::tuple<double, size_t, size_t> const &firstPair, std::tuple<double, size_t, size_t> const &secondPair) {
                  return std::get<0>(firstPair) > std::get<0>(secondPair);
              });

    // Get the pairs whose score is above fuzzyThresh
    if (numPairs == 0)
        numPairs = (unsigned int) allScores.size();

    count = 0;

    while (count < numPairs and count < allScores.size()) {
        if (std::get<0>(validPairs[count]) < fuzzyThresh)
            break;

        count++;
    }

    validPairs.resize(count);

    // Find return values
    std::vector<double> finalFirstCoords{};
    std::vector<double> finalSecondCoords{};

    std::vector<double> finalFirstAmps{};
    std::vector<double> finalSecondAmps{};

    std::vector<double> finalScores{};

    for (const auto &pair : validPairs) {
        size_t first = std::get<1>(pair);
        size_t second = std::get<2>(pair);

        finalFirstCoords.push_back(firstCoords[first]);
        finalSecondCoords.push_back(secondCoords[second]);

        finalFirstAmps.push_back(firstAmps[first]);
        finalSecondAmps.push_back(secondAmps[second]);

        finalScores.push_back(std::get<0>(pair));
    }

    // Find row and column coordinates of first edges, second edges, and center points between each pair
    auto firstPosDist = findEdgePos(measureHandle, finalFirstCoords);
    auto &finalFirstPos = firstPosDist.pos;

    auto secondPosDist = findEdgePos(measureHandle, finalSecondCoords);
    auto &finalSecondPos = secondPosDist.pos;

    std::vector<cv::Point2d> finalCenterPos;

    for (int i = 0; i < validPairs.size(); i++)
        finalCenterPos.push_back((finalFirstPos[i] + finalSecondPos[i]) / 2.0);

    // Find intra distance of each pair
    std::vector<double> intraDist{};
    std::vector<double> interDist{};

    for (int i = 0; i < validPairs.size(); i++)
        intraDist.push_back(findDistance(measureHandle, finalFirstCoords[i], finalSecondCoords[i]));

    return FuzzyMeasurePairsResult{finalFirstPos, finalFirstAmps,
                                   finalSecondPos, finalSecondAmps,
                                   intraDist, interDist,
                                   finalCenterPos, finalScores
    };
}

#endif
