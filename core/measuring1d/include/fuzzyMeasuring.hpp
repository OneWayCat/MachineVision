/**
 * @file fuzzyMeasuring.hpp
 * @brief Functions for 1D fuzzy measuring.
 */

#ifndef MEASURING_FUZZYMEASURING_HPP
#define MEASURING_FUZZYMEASURING_HPP

#include "measuring.hpp"

/**
 * @brief A structure to represent the results of the fuzzyMeasurePos function.
 */
struct FuzzyMeasurePosResult : MeasurePosResult {
    // Constructor
    FuzzyMeasurePosResult(PosDistanceResult &pos, std::vector<double> &amplitudes_,
                          std::vector<double> &scores_);

    /** Fuzzy score of each edge. */
    std::vector<double> fuzzyScores;
};

/**
 * @brief A structure to represent the results of the fuzzyMeasurePairs function.
 */
struct FuzzyMeasurePairsResult : MeasurePairsResult {
    // Default constructor
    FuzzyMeasurePairsResult() = default;

    // Constructor
    FuzzyMeasurePairsResult(std::vector<cv::Point2d> &posFirst, std::vector<double> &ampFirst,
                            std::vector<cv::Point2d> &posSecond, std::vector<double> &ampSecond,
                            std::vector<double> &intraDist, std::vector<double> &interDist,
                            std::vector<cv::Point2d> &posCenter, std::vector<double> &scores);

    /** Position of the center of each pair */
    std::vector<cv::Point2d> posCenter;

    /** Fuzzy score of each pair */
    std::vector<double> fuzzyScores;
};

/**
 * Since the results of the fuzzyMeasurePairings function and the fuzzyMeasurePairs are almost identical (minus interDistance),
 * we use the same struct for both of them, but interDistance is empty for fuzzyMeasurePairings.
 */
typedef FuzzyMeasurePairsResult FuzzyMeasurePairingsResult;

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
                double fuzzyThresh, TransitionType transition = TransitionType::ALL);

template<typename T>
FuzzyMeasurePosResult
fuzzyMeasurePos(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
                double fuzzyThresh, double cx, double cy, TransitionType transition = TransitionType::ALL);


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
                  double fuzzyThresh, TransitionType transition = TransitionType::ALL);

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
                     double fuzzyThresh, TransitionType transition = TransitionType::ALL, unsigned int numPairs = 0);

#endif //MEASURING_FUZZYMEASURING_HPP
