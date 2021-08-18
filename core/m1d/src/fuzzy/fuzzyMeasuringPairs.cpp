#include "fuzzyMeasuring.hpp"
#include "mutils.hpp"

using namespace cv;

/**
 * Return the score of a single edge pair based on the fuzzy functions.
 */
double getScoreSinglePair(const Mat &profile, const MeasureHandle &measureHandle, double firstAmp, double firstDist,
                          double secondAmp, double secondDist, double firstPairCenter, double lastPairCenter,
                          double firstEdgeDist, double lastEdgeDist) {

    if (secondDist <= firstDist)
        return -1;  // Invalid pair

    double score = 1.0;
    double centerDist = (firstDist + secondDist) / 2.0;

    double profileLength = measureHandle.getProfileLength();
    double halfProfile = profileLength / 2.0;

    for (const auto &it : measureHandle.getFuzzySet()) {
        // X Values for the fuzzy function
        // XValSecond is only for when the fuzzy score depends on the score of both of the edges of the pair
        // If xValSecond is an actual value, the geometric mean of xValFirst and second will be taken.
        double xValFirst = 0;
        double xValSecond = std::nan("0");

        // Determine the x value for the fuzzy function
        switch (it.first) {
            // Contrast
            case FuzzyType::CONTRAST:
                xValFirst = abs(firstAmp);
                xValSecond = abs(secondAmp);
                break;

                // Position pair
            case FuzzyType::POSITION_PAIR:
                xValFirst = centerDist;
                break;

            case FuzzyType::POSITION_FIRST_PAIR:
                xValFirst = centerDist - firstPairCenter;
                break;

            case FuzzyType::POSITION_LAST_PAIR:
                xValFirst = centerDist - lastPairCenter;
                break;

            case FuzzyType::POSITION_PAIR_CENTER:
                xValFirst = centerDist - halfProfile;
                break;

            case FuzzyType::POSITION_PAIR_END:
                xValFirst = centerDist - profileLength;
                break;

                // Position
            case FuzzyType::POSITION:
                xValFirst = firstDist;
                xValSecond = secondDist;
                break;

            case FuzzyType::POSITION_FIRST_EDGE:
                xValFirst = firstDist - firstEdgeDist;
                xValSecond = secondDist - firstEdgeDist;
                break;

            case FuzzyType::POSITION_LAST_EDGE:
                xValFirst = firstDist - lastEdgeDist;
                xValSecond = secondDist - lastEdgeDist;
                break;

            case FuzzyType::POSITION_CENTER:
                xValFirst = firstDist - halfProfile;
                xValSecond = secondDist - halfProfile;
                break;

            case FuzzyType::POSITION_END:
                xValFirst = firstDist - profileLength;
                xValSecond = secondDist - profileLength;
                break;

                // Size
            case FuzzyType::SIZE:
                xValFirst = secondDist - firstDist;
                break;

            case FuzzyType::SIZE_DIFF:
                xValFirst = measureHandle.getPairSize() - (secondDist - firstDist);
                break;

            case FuzzyType::SIZE_ABS_DIFF:
                xValFirst = abs(measureHandle.getPairSize() - (secondDist - firstDist));
                break;

                // Gray
            case FuzzyType::GRAY:
                int count = 0;
                double totalGrayValue = 0;
                auto profptr = profile.ptr<double>();

                for (int i = (int) ceil(firstDist); i <= (int) floor(secondDist) and i < profile.cols; i++) {
                    totalGrayValue += profptr[i];
                    count++;
                }

                xValFirst = totalGrayValue / count;
        }

        if (isnan(xValSecond))
            score *= it.second.interpolate(xValFirst);

        else {
            // Take the geometric mean of the two scores
            double firstScore = it.second.interpolate(xValFirst);
            double secondScore = it.second.interpolate(xValSecond);
            score *= sqrt(firstScore * secondScore);
        }
    }

    size_t numFuncs = measureHandle.getFuzzySet().size();

    // Take the geometric mean to get the overall score
    if (numFuncs > 0)
        score = pow(score, 1.0 / double(numFuncs));

    return score;
}

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
getScoresAllPairs(const Mat &profile, const MeasureHandle &measureHandle, const std::vector<double> &firstAmps,
                  const std::vector<double> &firstCoords, const std::vector<double> &secondAmps,
                  const std::vector<double> &secondCoords) {

    // Calculate distance from the left side of the profile line
    double profileLength = measureHandle.getProfileLength();
    size_t numberOfBins = measureHandle.numberOfBins();

    std::vector<double> firstDists = firstCoords;
    std::vector<double> secondDists = secondCoords;

    if (numberOfBins >= 2) {
        double distPerBin = profileLength / (numberOfBins - 1);

        for (int i = 0; i < firstCoords.size(); i++)
            firstDists[i] *= distPerBin;

        for (int i = 0; i < secondCoords.size(); i++)
            secondDists[i] *= distPerBin;
    }

    // Get the center coordinate of the first pair
    double firstPairLeft = firstDists.front();
    double firstPairRight = -1;

    for (const double &secondDist : secondDists) {
        if (secondDist > firstPairLeft) {
            firstPairRight = secondDist;
            break;
        }
    }

    // No valid pairs
    if (firstPairRight == -1)
        return std::vector<double>{};

    double firstPairCenter = (firstPairLeft + firstPairRight) / 2.0;

    // Get the center coordinate of the last pair
    double lastPairRight = secondDists.back();
    double lastPairLeft = -1;

    for (auto it = firstDists.rbegin(); it != firstDists.rend(); it++) {
        if (*it < lastPairRight) {
            lastPairLeft = *it;
            break;
        }
    }

    double lastPairCenter = (lastPairLeft + lastPairRight) / 2.0;

    // Get the coordinates of the first and last edge
    double firstEdgeDist = std::min(firstDists.front(), secondDists.front());
    double lastEdgeDist = std::max(firstDists.back(), secondDists.back());

    // 2D array to store all scores
    std::vector<double> allScores{};

    for (int i = 0; i < firstDists.size(); i++) {
        for (int j = 0; j < secondDists.size(); j++) {
            allScores.push_back(getScoreSinglePair(profile, measureHandle, firstAmps[i], firstDists[i], secondAmps[j],
                                                   secondDists[j], firstPairCenter, lastPairCenter, firstEdgeDist,
                                                   lastEdgeDist));
        }
    }

    return allScores;
}

/*
 * Find the (rowIndex, colIndex, score) of the elements of the scores matrix that match maxScore.
 */
std::vector<std::tuple<int, int, double>>
findMaxPairs(const std::vector<double> &scores, size_t nRows, size_t nCols, double maxScore) {
    std::vector<std::tuple<int, int, double>> maxPairs{};

    for (size_t r = 0; r < nRows; r++) {
        for (size_t c = 0; c < nCols; c++) {
            double score = scores[r * nCols + c];

            if (score == maxScore)
                maxPairs.emplace_back(int(r), int(c), score);
        }
    }

    return maxPairs;
}

std::vector<std::tuple<size_t, size_t, double>>
findValidPairs(std::vector<double> &allScores, size_t nRows, size_t nCols, double fuzzyThresh,
               const std::vector<double> &firstCoords, const std::vector<double> &secondCoords) {
    // The array of all pairs that we will return
    // Each element is a tuple of (firstIndex, secondIndex, score)
    std::vector<std::tuple<size_t, size_t, double>> validPairs{};

    // Find the appropriate edge pairs from the scores matrix
    while (true) {
        // Find the global maximum
        double maxScore = *std::max_element(allScores.begin(), allScores.end());

        // No more pairs with score above threshold, we found all pairs
        if (maxScore < fuzzyThresh)
            return validPairs;

        // Among the pairs with score == maxScore, find the (rowIndex, colIndex, score) of each pair
        auto maxPairs = findMaxPairs(allScores, nRows, nCols, maxScore);

        // Find the minimum intra distance (colIndex - rowIndex) out of these pairs
        int minIntraDist = int(nCols);

        for (const auto &pair : maxPairs) {
            int intraDist = abs(std::get<1>(pair) - std::get<0>(pair));

            if (intraDist < minIntraDist)
                minIntraDist = intraDist;
        }

        // Find the first pair with the minimum intra distance that we just found
        int foundFirstIndex = 0;
        int foundSecondIndex = 0;

        for (const auto &pair : maxPairs) {
            int pairFirstIndex = std::get<0>(pair);
            int pairSecondIndex = std::get<1>(pair);
            int intraDist = abs(pairSecondIndex - pairFirstIndex);

            if (intraDist == minIntraDist) {
                foundFirstIndex = pairFirstIndex;
                foundSecondIndex = pairSecondIndex;

                validPairs.push_back(pair);
                break;
            }
        }

        // Set the scores of all invalid pairs to -1 now
        // First, the ones that would overlap the found pair from the left side
        for (size_t c = nCols; c > 0; c--) {
            size_t col = c - 1;

            if (secondCoords[col] < firstCoords[foundFirstIndex])
                break;

            for (size_t r = 0; r < foundFirstIndex; r++)
                allScores[r * nCols + col] = -1;
        }

        // Then, the ones that would overlap the found pair from the right side
        for (size_t r = foundFirstIndex; r < nRows; r++) {
            if (firstCoords[r] > secondCoords[foundSecondIndex])
                break;

            for (size_t c = 0; c < nCols; c++)
                allScores[r * nCols + c] = -1;
        }
    }
}

template<typename T>
FuzzyMeasurePairsResult
fuzzyMeasurePairs(const Mat &img, const T &measureHandle, double sigma, double ampThresh,
                  double fuzzyThresh, TransitionType transition) {

    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);

    Mat profile = measureProjection(img, measureHandle);

    // We create a copy because we need the original profile to score the pairs for FuzzyType::GRAY
    Mat profileCopy = profile.clone();

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

    std::vector<Point2d> finalCenterPos{};

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

template FuzzyMeasurePairsResult
fuzzyMeasurePairs<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                    double ampThresh, double fuzzyThresh, TransitionType transition);

template FuzzyMeasurePairsResult
fuzzyMeasurePairs<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
                              double fuzzyThresh, TransitionType transition);

/* I could have wrote helper methods to avoid duplicate code between fuzzyMeasurePairs and this method,
 * but I've decided that it would decrease readability too much for it to be beneficial. */
template<typename T>
FuzzyMeasurePairingsResult
fuzzyMeasurePairings(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                     double fuzzyThresh, TransitionType transition, unsigned int numPairs) {

    // Validate fuzzyThresh
    if (fuzzyThresh < 0 or fuzzyThresh > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    validateArgs(img, sigma, ampThresh);

    Mat profile = measureProjection(img, measureHandle);

    // We create a copy because we need the original profile to score the pairs for FuzzyType::GRAY
    Mat profileCopy = profile.clone();

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

template FuzzyMeasurePairingsResult
fuzzyMeasurePairings<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                       double ampThresh, double fuzzyThresh, TransitionType transition,
                                       unsigned int numPairs);

template FuzzyMeasurePairsResult
fuzzyMeasurePairings<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
                                 double fuzzyThresh, TransitionType transition, unsigned int numPairs);