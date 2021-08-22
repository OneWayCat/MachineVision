

#ifndef MEASURING_EDGEELEMENT_HPP
#define MEASURING_EDGEELEMENT_HPP

#include <unordered_map>

#include "MeasuringResults.hpp"
#include "FuzzyStructs.hpp"

struct EnumClassHash {
    template<typename T>
    std::size_t operator()(T t) const {
        return static_cast<std::size_t>(t);
    }
};

/**
 * @brief Base class of a measure object for extracting straight edges perpendicular to
 * a rectangle or an annular arc.
 */
class EdgeElement {
protected:
    int row;  // Row coordinate of the center
    int column;  // Column coordinate of the center

    std::unordered_map<FuzzyType, FuzzyFunction, EnumClassHash> fuzzySet;  // Fuzzy functions to use
    double pairSize;  // For normalized fuzzy function

    // Minimum and maximum coordinates for the bounding box of the handle
    // These values will be used in MeasureProjection
    int minC{};
    int maxC{};
    int minR{};
    int maxR{};

    double profileLength{};  // Length of the profile line

    std::vector<int> binNumbers;  // 2D array to store bin numbers for each (r, c) pair
    std::vector<int> binCounts;  // 1D array to store number of pixels per each bin

    EdgeElement(int row, int column);  // Constructor

protected:

    /**
     * @brief Find the row and column coordinates of the points specified by the coords vector inside the
     * edge element and the distances between each pair of consecutive points found.
     *
     * @param coords A vector that specifies where the points are inside measureHandle.
     * @param select Whether to return all points or just the first or last point.
     * @return A structure representing the edgePositions of the points and the distance between each pair of consecutive points.
     */
    virtual EdgeResults findEdgePos(const std::vector<double> &coords, SelectType select = SelectType::ALL) = 0;

    // Find the distance between 2 edges in an edge element.
    virtual double findDistance(double firstCoord, double secondCoord) = 0;

public:
    /**
     * @fn cv::Mat projectOnToProfileLine(const cv::Mat &img, const MeasureHandle &measureHandle)
     * @brief Extract a one-dimensional gray value profile perpendicular to a rectangle or annular arc.
       @details This is done by taking the mean of the 2D image across the axis perpendicular to the major axis of
        the rectangle or the annular arc.

     * @param img Single-channel input image
     * @param measureHandle The MeasureHandle object. This determines the region of interest.
     * @return The one-dimensional gray-value profile.
     */
    cv::Mat projectOnToProfileLine(const cv::Mat &img) const;

    /**
     * @brief Get the row coordinate of the center of the handle.
     * @return row
     */
    int getRow() const { return row; }

    /**
     * @brief Get the column coordinate of the center of the handle.
     * @return column
     */
    int getColumn() const { return column; }

    /**
     * @brief Get the fuzzy functions that the object is using
     * @return fuzzySet
     */
    const std::unordered_map<FuzzyType, FuzzyFunction, EnumClassHash> &getFuzzySet() const { return fuzzySet; }

    /**
     * @brief Set the fuzzy type and its fuzzy function in the measure handle to score edges or edge pairs with.
     * @param func Fuzzy function to use to score edges with.
     * @param type Fuzzy criteria to use.
     */
    void setFuzzyFunc(const FuzzyFunction &func, FuzzyType type);

    /**
     * @brief Set the fuzzy type and its normalized fuzzy function in the measure handle.
     * @details Operates the same as setFuzzyFunc, but the x coordinates are scaled (multiplied)
     *  by pairSize before being put into the MeasureHandle.
     *
     * @param func Fuzzy function to use to score edges with.
     * @param type Fuzzy criteria to use.
     * @param pairSize The normalizing factor for the fuzzy function.
     */
    void setNormFuzzyFunc(const FuzzyFunction &func, FuzzyType type, double pairSize_);

    /**
     * @brief Remove a fuzzy function for the speficied fuzzy type.
     * @param type The fuzzy type to remove.
     */
    void removeFuzzyFunc(FuzzyType type) { fuzzySet.erase(type); }

    /**
     * @brief Get the pair size for the fuzzy functions.
     * @return pairSize
     */
    double getPairSize() const { return pairSize; }

    /**
     * @brief Translate a measure object.
     * @param newRow
     * @param newCol
     */
    void translateMeasure(int newRow, int newCol);

    size_t numberOfBins() const { return binCounts.size(); }

    double getProfileLength() const { return profileLength; }
};

#endif
