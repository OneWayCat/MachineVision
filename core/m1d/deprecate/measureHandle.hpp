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