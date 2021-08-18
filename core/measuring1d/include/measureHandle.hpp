/**
 * @file measureHandle.hpp
 * @brief MeasureHandle classes for 1D measuring.
 */

#ifndef MEASURING_MEASUREHANDLE_HPP
#define MEASURING_MEASUREHANDLE_HPP

#include <ciso646>
#include <cmath>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <boost/math/constants/constants.hpp>

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
class FuzzyFunc {
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
    FuzzyFunc(const std::vector<double> &xVals, const std::vector<double> &yVals);

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
class MeasureHandle {
protected:
    int row;  // Row coordinate of the center
    int column;  // Column coordinate of the center

    std::unordered_map<FuzzyType, FuzzyFunc, EnumClassHash> fuzzySet;  // Fuzzy functions to use
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

    MeasureHandle(int row, int column);  // Constructor

    virtual ~MeasureHandle() = 0;  // Destructor

public:
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
    const std::unordered_map<FuzzyType, FuzzyFunc, EnumClassHash> &getFuzzySet() const { return fuzzySet; }

    /**
     * @brief Set the fuzzy type and its fuzzy function in the measure handle to score edges or edge pairs with.
     * @param func Fuzzy function to use to score edges with.
     * @param type Fuzzy criteria to use.
     */
    void setFuzzyFunc(const FuzzyFunc &func, FuzzyType type);

    /**
     * @brief Set the fuzzy type and its normalized fuzzy function in the measure handle.
     * @details Operates the same as setFuzzyFunc, but the x coordinates are scaled (multiplied)
     *  by pairSize before being put into the MeasureHandle.
     *
     * @param func Fuzzy function to use to score edges with.
     * @param type Fuzzy criteria to use.
     * @param pairSize The normalizing factor for the fuzzy function.
     */
    void setNormFuzzyFunc(const FuzzyFunc &func, FuzzyType type, double pairSize_);

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

    // (for internal use)
    cv::Mat measureProjection_(const cv::Mat &img) const;

    cv::Mat measureProjection_(const cv::Mat &img, double cx, double cy) const;

    size_t numberOfBins() const { return binCounts.size(); }

    double getProfileLength() const { return profileLength; }
};

/**
 * @brief A measure object for extracting straight edges perpendicular to a rectangle.
 */
class MeasureRectangle : public MeasureHandle {
private:
    int halfWidth;  // Half of the width of the rectangle
    int halfHeight;  // Half of the height of the rectangle

    double phi;  // Angle of the orientation of the rectangle

public:
    /**
     * @brief Construct a MeasureRectangle object.
     *
     * @param halfWidth Half of the width of the rectangle.
     * @param halfHeight Half of the height of the rectangle.
     * @param row Row coordinate of the center.
     * @param column Column coordinate of the center.
     * @param phi Orientation angle of the rectangle (counterclockwise).
     * @param angleInDegrees Whether phi is in degrees or radians.
     */
    MeasureRectangle(int halfWidth, int halfHeight, int row, int column, double phi = 0,
                     bool angleInDegrees = false);

    /**
     * @brief Get the half width of the rectangle
     * @return halfWidth
     */
    int getHalfWidth() const { return halfWidth; }

    /**
     * @brief Get the half height of the rectangle
     * @return halfHeight
     */
    int getHalfHeight() const { return halfHeight; }

    /**
     * @brief Get the orientation angle of the rectangle
     * @return phi
     */
    double getPhi() const { return phi; }
};

/**
 * @brief A measure object for extrating straight edges perpendicular to an annular arc.
 */
class MeasureArc : public MeasureHandle {
private:
    int radius;  // Radius of the arc
    int innerRadius;  // Radius of the annulus

    double angleStart;  // Starting angle of the arc
    double angleExtent;  // Angular extent of the arc

public:
    /**
     * @brief Construct a MeasureArc object.
     *
     * @param radius Outer radius of the arc.
     * @param innerRadius Radius of the annulus. \n
     * Range of values: 0 <= innerRadius < radius
     *
     * @param row Row coordinate of the center.
     * @param column Column coordinate of the center.
     * @param angleStart Starting angle of the arc, measured from the positive horizontal axis counterclockwise \n
     * Range of values: -pi < angleStart <= pi \n
     * Default value: 0
     *
     * @param angleExtent Angular extent of the arc \n
     * Range of values: -2pi <= angleExtent <= 2pi \n
     * Default value: 2pi
     *
     * @param angleInDegrees Whether angleStart and angleExtent are given in degrees.
     */
    MeasureArc(int radius, int innerRadius, int row, int column, double angleStart = 0,
               double angleExtent = boost::math::double_constants::two_pi,
               bool angleInDegrees = false);

    /**
     * @brief Get the outer radius of the annular arc
     * @return radius
     */
    int getRadius() const { return radius; }

    /**
     * @brief Get the radius of the annulus
     * @return innerRadius
     */
    int getInnerRadius() const { return innerRadius; }

    /**
     * @brief Get the starting angle of the arc
     * @return angleStart
     */
    double getAngleStart() const { return angleStart; }

    /**
     * @brief Get the angle extent of the arc
     * @return angleExtent
     */
    double getAngleExtent() const { return angleExtent; }
};

class MeasureArcTransposed : public MeasureHandle {
private:
    int radius;  // Radius of the arc
    int innerRadius;  // Radius of the annulus

    double angleStart;  // Starting angle of the arc
    double angleExtent;  // Angular extent of the arc

public:
    /**
     * @brief Construct a MeasureArc object.
     *
     * @param radius Outer radius of the arc.
     * @param innerRadius Radius of the annulus. \n
     * Range of values: 0 <= innerRadius < radius
     *
     * @param row Row coordinate of the center.
     * @param column Column coordinate of the center.
     * @param angleStart Starting angle of the arc, measured from the positive horizontal axis counterclockwise \n
     * Range of values: -pi < angleStart <= pi \n
     * Default value: 0
     *
     * @param angleExtent Angular extent of the arc \n
     * Range of values: -2pi <= angleExtent <= 2pi \n
     * Default value: 2pi
     *
     * @param angleInDegrees Whether angleStart and angleExtent are given in degrees.
     */
    MeasureArcTransposed(int radius, int innerRadius, int row, int column, double angleStart = 0,
               double angleExtent = boost::math::double_constants::two_pi,
               bool angleInDegrees = false);

    /**
     * @brief Get the outer radius of the annular arc
     * @return radius
     */
    int getRadius() const { return radius; }

    /**
     * @brief Get the radius of the annulus
     * @return innerRadius
     */
    int getInnerRadius() const { return innerRadius; }

    /**
     * @brief Get the starting angle of the arc
     * @return angleStart
     */
    double getAngleStart() const { return angleStart; }

    /**
     * @brief Get the angle extent of the arc
     * @return angleExtent
     */
    double getAngleExtent() const { return angleExtent; }
};

#endif