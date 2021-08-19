#include "FuzzyMeasuring.hpp"

std::pair<std::vector<double>, std::vector<size_t>>
getScoresPos(const EdgeElement &measureHandle, const std::vector<double> &amplitudes, const std::vector<double> &coords, double fuzzyThresh) {

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

double getScoreSinglePair(const cv::Mat &profile, const EdgeElement &measureHandle, double firstAmp, double firstDist,
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

std::vector<double>
getScoresAllPairs(const cv::Mat &profile, const EdgeElement &measureHandle, const std::vector<double> &firstAmps,
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
findValidPairs(std::vector<double> &allScores, size_t nRows, size_t nCols, double fuzzyThresh, const std::vector<double> &firstCoords, const std::vector<double> &secondCoords) {
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