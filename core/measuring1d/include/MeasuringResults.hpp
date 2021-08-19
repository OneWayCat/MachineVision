

#ifndef MEASURING_MEASURINGRESULTS_HPP
#define MEASURING_MEASURINGRESULTS_HPP

#include "opencv2/core.hpp"

/**
 * @enum TransitionType
 * @brief Whether to extract all edges, or just light->dark or dark->light edges.
 */
enum class TransitionType {
    ALL,
    POSITIVE,
    NEGATIVE
};
/**
 * @enum SelectType
 * @brief Select all edges detected, or just the first or last edge.
 */
enum class SelectType {
    ALL,
    FIRST,
    LAST
};

/**
 * @brief A structure to represent the edgePositions of each edge and the distances between consecutive edges.
 *
 * The purpose of the result structs is to return information about the measurements in the measuring functions, so
 * move semantics are used instead of copying to avoid copying data when returning the information.
 * std::move is called on the parameters to the constructor, so they should not be used after calling the constructor.
 */
struct EdgeResults {
    // Default constructor
    EdgeResults() = default;

    // Constructor
    EdgeResults(std::vector<cv::Point2d> &pos_, std::vector<double> &distances_);

    // Copy constructor
    EdgeResults(const EdgeResults &obj) = default;

    // Move constructor
    EdgeResults(EdgeResults &&obj) noexcept;

    /** Position of each edge represented as (row, column) points */
    std::vector<cv::Point2d> edgePositions;

    /** Distances between consecutive edges (empty if select is not 'all'). */
    std::vector<double> distances;
};

/**
 * @brief A structure to represent the results of the measurePos function.
 */
struct EdgePositionResults : EdgeResults {
    // Constructor
    EdgePositionResults(EdgeResults &pos, std::vector<double> &amplitudes_);

    /** Signed amplitude of each edge. */
    std::vector<double> amplitudes;
};

/**
 * @brief A structure to represent the results of the measurePairs function.
 */
struct EdgePairsResults {
    // Default constructor
    EdgePairsResults() = default;

    // Constructor
    EdgePairsResults(std::vector<cv::Point2d> &posFirst_, std::vector<double> &amplitudesFirst_,
                     std::vector<cv::Point2d> &posSecond_, std::vector<double> &amplitudesSecond_,
                     std::vector<double> &intraDistance_, std::vector<double> &interDistance_);

    /** Position (row, column) of the first edge of each pair. */
    std::vector<cv::Point2d> posFirst;
    /** Amplitude of the first edge of each pair. */
    std::vector<double> amplitudesFirst;

    /** Position of the second edge of each pair. */
    std::vector<cv::Point2d> posSecond;
    /** Amplitude of the second edge of each pair. */
    std::vector<double> amplitudesSecond;

    /** Distance between the edges of each pair. */
    std::vector<double> intraDistance;
    /** Distance between consecutive edge pairs. */
    std::vector<double> interDistance;

    // Return a EdgePairsResults struct with just the first element of each data field
    EdgePairsResults front() const;

    // Return a EdgePairsResults struct with just the last element of each data field
    EdgePairsResults back() const;
};

#endif
