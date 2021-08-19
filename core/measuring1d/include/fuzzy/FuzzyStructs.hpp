
#ifndef MEASURING_FUZZYSTRUCTS_HPP
#define MEASURING_FUZZYSTRUCTS_HPP

#include "MeasuringResults.hpp"

/**
 * @enum FuzzyType
 * @brief The criteria to score all edges found in the MeasureHandle object.
 */
enum class FuzzyType {
    /** Score based on unsigned edge amplitude. For pairs, the geometric mean of the two scores is taken.
     * Not available for normalized fuzzy function. */
    CONTRAST,
    /** Score based on the mean gray value between the two edges of the pair. Only used for fuzzyMeasurePairs/Pairing.
     * Not available for normalized fuzzy function. */
    GRAY,

    /** Score based on distance from the beginning of the profile line. For pairs, the geometric mean of the
     * two scores is taken. */
    POSITION,
    /** Same as FuzzyType::POSITION, but with distance from the first edge. */
    POSITION_FIRST_EDGE,
    /** Same as FuzzyType::POSITION, but with distance from the last edge. */
    POSITION_LAST_EDGE,
    /** Same as FuzzyType::POSITION, but with distance from the center of the profile line. */
    POSITION_CENTER,
    /** Same as FuzzyType::POSITION, but with distance from the end of the profile line. */
    POSITION_END,

    /** Score based on distance from the beginning of the profile line. The position of the pair is determined by the
     * center point between the two edges. Only used for fuzzyMeasurePairs/Pairing. */
    POSITION_PAIR,
    /** Same as FuzzyType::POSITION_PAIR, but with distance from the first pair. */
    POSITION_FIRST_PAIR,
    /** Same as FuzzyType::POSITION_PAIR, but with distance from the last pair. */
    POSITION_LAST_PAIR,
    /** Same as FuzzyType::POSITION_PAIR, but with distance from the center of the profile line. */
    POSITION_PAIR_CENTER,
    /** Same as FuzzyType::POSITION_PAIR, but with distance from the end of the profile line. */
    POSITION_PAIR_END,

    /** Score based on the size of the edge pair, which is the distance between the first and second edge of the pair. */
    SIZE,
    /** Score based on normalized size difference. Only available for normalized fuzzy function. */
    SIZE_DIFF,
    /** Score based on the absolute value of normalized size difference. Only available for normalized fuzzy function. */
    SIZE_ABS_DIFF
};

/**
 * @brief A class representing a piecewise linear function for fuzzy scoring.
 */
class FuzzyFunction {
private:
    std::vector<double> xVals;
    std::vector<double> yVals;

public:
    /**
     * @brief Create a fuzzyFunc object. At least two (x, y) pairs must be given.
     * @param xVals A vector of the x coordinates to create the piecewise function with. \n
     * Restrictions: This vector must be in ascending order and cannot have duplicates.
     *
     * @param yVals A vector of the x coordinates to create the piecewise function with. \n
     * Restrictions: The elements of this vector must be between 0 and 1 (inclusive).
     */
    FuzzyFunction(const std::vector<double> &xVals, const std::vector<double> &yVals);

    /**
     * @brief Get the x coordinates of the function.
     * @return xVals
     */
    const std::vector<double> &getXVals() const { return xVals; }

    /**
     * @brief Get the y coordinates of the function.
     * @return yVals
     */
    const std::vector<double> &getYVals() const { return yVals; }

    /**
     * @brief Interpolate the y value of the fuzzy function given the x value.
     * @param x Value to interpolate at.
     * @return The y value interpolated.
     */
    double interpolate(double x) const;

    /**
     * @brief Multiply each x coordinate by factor
     * @param factor
     * @throws std::invalid_argument if factor is negative
     */
    void scale(double factor);
};

/**
 * @brief A structure to represent the results of the fuzzyMeasurePos function.
 */
struct FuzzyMeasurePosResult : EdgePositionResults {
    // Constructor
    FuzzyMeasurePosResult(EdgeResults &pos, std::vector<double> &amplitudes_,
                          std::vector<double> &scores_);

    /** Fuzzy score of each edge. */
    std::vector<double> fuzzyScores;
};

/**
 * @brief A structure to represent the results of the fuzzyMeasurePairs function.
 */
struct FuzzyMeasurePairsResult : EdgePairsResults {
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
#endif
