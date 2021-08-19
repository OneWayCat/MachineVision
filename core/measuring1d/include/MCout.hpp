

#ifndef MEASURING_MCOUT_HPP
#define MEASURING_MCOUT_HPP

#include "FuzzyMeasuring.hpp"

/**
 * @fn std::ostream& operator<<(std::ostream& o, const MeasurePosResult& p)
 * @brief Overloads cout to accept MeasurePosResult struct as input
 * @details Prints (row, column, amplitude) as a single array
            and then prints distances in a different array
 *
 * @param out The ostream object
 * @param p The MeasurePosResult struct to print
 */
std::ostream &operator<<(std::ostream &o, const MeasurePosResults &p);

/**
 * @fn std::ostream& operator<<(std::ostream& o, const MeasurePairsResult& p)
 * @brief Overloads cout to accept MeasurePairsResult struct as input
 * @details Prints (row1, col1, amp1) --- (row2, col2, amp2), then intraDist, and finally interDist
            as three separate arrays
 *
 * @param out The ostream object
 * @param p The MeasurePairsResult struct to print
 */
std::ostream &operator<<(std::ostream &o, const MeasurePairsResults &p);

/**
 * @fn std::ostream& operator<<(std::ostream& o, const FuzzyMeasurePosResult& p)
 * @brief Overloads cout to accept FuzzyMeasurePosResult struct as input
 * @details Prints (row, col, amp, score) as one array and then distances in a different array
 *
 * @param out The ostream object
 * @param p The FuzzyMeasurePosResult struct to print
 */
std::ostream &operator<<(std::ostream &o, const FuzzyMeasurePosResult &p);

/**
 * @fn std::ostream& operator<<(std::ostream& o, const FuzzyMeasurePairsResult& p)
 * @brief Overloads cout to accept FuzzyMeasurePairsResult struct as input
 * @details Prints (row, col, amp, score) as one array,
 *          then prints (rowCenter, colCenter) as a different array.
 *          then intra dist and finally inter dist.
 *
 *
 * @param out The ostream object
 * @param p The FuzzyMeasurePairsResult struct to print
 */
std::ostream &operator<<(std::ostream &o, const FuzzyMeasurePairsResult &p);

#endif
