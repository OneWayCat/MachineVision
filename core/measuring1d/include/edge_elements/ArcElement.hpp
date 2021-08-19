
#ifndef MEASURING_ARCELEMENT_HPP
#define MEASURING_ARCELEMENT_HPP

#include "EdgeElement.hpp"

/**
 * Determine if the point (r, c) is within the annular arc
 *
 * @param innerRadius
 * @param radius
 * @param angleStart
 * @param angleEnd
 * @param pointNorm norm of the point to test
 * @param pointAngle angle of the point to test
 * @return Whether the point is within the arc
 */
bool insideArc(int innerRadius, int radius, double angleStart, double angleEnd, double pointNorm, double pointAngle);


/**
 * @brief A measure object for extrating straight edges perpendicular to an annular arc.
 */
class ArcElement : public virtual EdgeElement {
protected:
    int radius;  // Radius of the arc
    int innerRadius;  // Radius of the annulus

    double angleStart;  // Starting angle of the arc
    double angleExtent;  // Angular extent of the arc

    EdgeResults findEdgePos(const std::vector<double> &coords, SelectType select = SelectType::ALL) override;

    double findDistance(double firstCoord, double secondCoord) override;

public:
    /**
     * @brief Construct a ArcElement object.
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
    ArcElement(int radius, int innerRadius, int row, int column, double angleStart = 0, double angleExtent = CV_2PI, bool angleInDegrees = false);

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
