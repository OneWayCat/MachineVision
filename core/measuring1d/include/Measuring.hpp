
#ifndef MEASURING_MEASURING_HPP
#define MEASURING_MEASURING_HPP

#include "MeasuringResults.hpp"

/* Find the subpixel position and the amplitude of the edges at that position given all the
 * edge amplitudes and the position of the local maxima/minima  */
std::pair<std::vector<double>, std::vector<double>> findSubpixelPos(const Mat &profile, const std::vector<int> &aboveThreshold);

std::pair<std::vector<double>, std::vector<double>> getEdgeAmplitudes(Mat &profile, const MeasureHandle &measureHandle, double sigma, double threshold, TransitionType transition)

bool betweenInclusive(double a, double b, double c);

/**
 * @fn measurePos(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
                            TransitionType transition, SelectType select)
   @brief Find the position of straight edges perpendicular to a rectangle or an annular arc.
   @details Initially, the region of interest is extracted from the image using the measure object.
    To extract the edges, gaussian smoothing is applied to pre-process the image and the
    derivative of the image is found by convolving the image with a derivative kernel. Then,
    a 1D array is generated by averaging the gray values in each row (or column).
    The function takes the edges that have an amplitude greater than the threshold
    value, and returns the positions and signed amplitudes of those edges.

 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing. \n
 *  Range of values: 0.4 <= sigma <= 100
 * @param threshold Minimum edge amplitude. \n
 *  Range of values: 0 <= threshold <= 255
 * @param transition Detect all edges, or just dark->light or light->dark edges.
 * @param select Return all edges found, or just the first or the last one.
 * @return A MeasurePosResult struct containing the row, column coordinate, amplitude of each edge and the distance
 *  between each consecutive edge.
 */
template<typename T>
MeasurePosResult measurePos(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
                            TransitionType transition = TransitionType::ALL, SelectType select = SelectType::ALL) {
    validateArgs(img, sigma, threshold);
    Mat profile = measureProjection(img, measureHandle);

    auto returnVal = getEdgeAmplitudes(profile, measureHandle, sigma, threshold, transition);
    std::vector<double> &aboveThreshold = returnVal.first;
    std::vector<double> &amplitudes = returnVal.second;

    if (!amplitudes.empty()) {
        if (select == SelectType::FIRST)
            amplitudes = getFirst(amplitudes);

        else if (select == SelectType::LAST)
            amplitudes = getLast(amplitudes);
    }

    PosDistanceResult pos = findEdgePos(measureHandle, aboveThreshold, select);
    return MeasurePosResult(pos, amplitudes);
}

/**
 * @fn measurePairs(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
             TransitionType transition = TransitionType::ALL, SelectType select = SelectType::ALL, bool pickStrongest = false);
 * @brief Find the position of straight edge pairs perpendicular to a rectangle or an annular arc.
 * @details The edge detection algorithm of this function is the same as that of measurePos, but
        this function finds edge pairs instead of individual edges. The function returns the
        position and the amplitude of each edge pair, as well as the distance between each
        edge pair and the distance between consecutive edge pairs.

 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing. \n
 *  Range of values: 0.4 <= sigma <= 100
 * @param threshold threshold Minimum edge amplitude. \n
 *  Range of values: 0 <= threshold <= 255
 * @param transition TransitionType::POSITIVE: dark->light->dark edge pairs are detected. \n
 * TransitionType::NEGATIVE: light->dark->light edge pairs are detected. \n
 * TransitionType::ALL: the detection is based on the first detected edge. If the first
                edge is a positive edge, then positive edge pairs will be detected, and
                vice versa.
 *
 * @param select Return all edge pairs found, or just the first or the last one.
 * @param pickStrongest If there are more than one consecutive edge with the same polarity, the
            first one will be used for the edge pair. But if pickStrongest is true, the one with
            the maximum amplitude will be used for the edge pair.
 * @return A MeasurePairsResult struct containing the row, column coordinate, amplitude of each edge pair, the distance
 *  between the edges of each pair and the distance between consecutive edge pairs.
 */
template<typename T>
MeasurePairsResult
measurePairs(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
             TransitionType transition = TransitionType::ALL, SelectType select = SelectType::ALL, bool pickStrongest = false) {
    // Detect all edges
    MeasurePosResult edges = measurePos(img, measureHandle, sigma, threshold);

    // Make sure there are at least 2 edges (for at least one edge pair)
    if (edges.amplitudes.size() < 2)
        return MeasurePairsResult{};

    // If transition is ALL, the first edge determines the polarity of edge pairs to detect
    if (transition == TransitionType::ALL)
        transition = edges.amplitudes.front() > 0 ? TransitionType::POSITIVE : TransitionType::NEGATIVE;

    // Iterate through edges to find the edge pairs
    int sign = transition == TransitionType::POSITIVE ? 0 : 1;

    int index = 0;                   // Current index
    std::vector<int> firstIndex{};   // Indices within amplitudes for the first edge of each edge pair
    std::vector<int> secondIndex{};  // Indices within amplitudes for the second edge of each edge pair

    while (true) {
        // Find the first edge of the pair
        int temp = nextSignedElement(edges.amplitudes, sign, index, pickStrongest);

        if (temp == -1)
            break;

        firstIndex.push_back(temp);
        index = temp + 1;

        // Find the second edge of the pair
        temp = nextSignedElement(edges.amplitudes, !sign, index, pickStrongest);

        if (temp == -1)
            break;

        secondIndex.push_back(temp);
        index = temp + 1;
    }

    // Make sure firstIndex and secondIndex have the same length
    firstIndex.resize(secondIndex.size());

    // Initialize vectors to return
    size_t n = firstIndex.size();

    auto posFirst = indexArray(edges.pos, firstIndex);
    auto amplitudesFirst = indexArray(edges.amplitudes, firstIndex);

    auto posSecond = indexArray(edges.pos, secondIndex);
    auto amplitudesSecond = indexArray(edges.amplitudes, secondIndex);

    // Find intraDistance and interDistance
    std::vector<double> intraDistance(n);
    std::vector<double> interDistance(n == 0 ? 0 : n - 1);

    auto distancePtr = edges.distances.cbegin();

    for (size_t i = 0; i < n; i++)
        intraDistance[i] = std::accumulate(distancePtr + firstIndex[i], distancePtr + secondIndex[i], 0.0);

    for (size_t i = 0; i < interDistance.size(); i++)
        interDistance[i] = std::accumulate(distancePtr + secondIndex[i], distancePtr + firstIndex[i + 1], 0.0);

    auto result = MeasurePairsResult{posFirst, amplitudesFirst, posSecond,
                                     amplitudesSecond, intraDistance, interDistance};

    // Return values
    switch (select) {
        case SelectType::ALL:
            return result;

        case SelectType::FIRST:
            return result.front();

        default:
            return result.back();
    }
}

/**
 * @fn PosDistanceResult measureThresh(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
                                SelectType select = SelectType::ALL)

 * @brief Find points in a measure handle with a particular gray value in a rectangle or annular arc.
 * @tparam T Either MeasureRectangle or MeasureArc.
 * @param img Single-channel input image.
 * @param measureHandle The MeasureHandle object. This determines the region of interest.
 * @param sigma Standard deviation for gaussian smoothing.
 * @param threshold Gray value to look for.
 * @param select Return all edges found, or just the first or the last one.
 * @return A struct containing the row and column coordinates of each point found and the distance
 *  between each consecutive points.
 */
template<typename T>
PosDistanceResult measureThresh(const cv::Mat &img, const T &measureHandle, double sigma, double threshold,
                                SelectType select = SelectType::ALL) {
    validateArgs(img, sigma, threshold);

    // Get the 1D profile
    Mat profile = measureProjection(img, measureHandle);

    // Gaussian smoothing before detecting edges
    GaussianBlur(profile, profile, Size(3, 1), sigma, 0, BORDER_REPLICATE);

    // Find the points where profile crosses the threshold
    std::vector<double> matchThreshold{};
    auto profptr = profile.ptr<double>();

    for (int i = 0; i < profile.cols; i++) {
        // Exact match
        if (threshold == profptr[i])
            matchThreshold.push_back(i);

        // Crosses the threshold
        if (i < profile.cols - 1 and betweenInclusive(profptr[i], profptr[i + 1], threshold)) {
            // Do linear interpolation
            double subpixel = (threshold - profptr[i]) / (profptr[i + 1] - profptr[i]);
            matchThreshold.push_back(i + subpixel);
        }
    }

    return findEdgePos(measureHandle, matchThreshold, select);
}

#endif
