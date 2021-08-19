
#ifndef MEASURING_ARCTRANSPOSEDELEMENT_HPP
#define MEASURING_ARCTRANSPOSEDELEMENT_HPP

#include "ArcElement.hpp"

class ArcTransposedElement : public virtual ArcElement {
protected:
    EdgeResults findEdgePos(const std::vector<double> &coords, SelectType select = SelectType::ALL) override;

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
    ArcTransposedElement(int radius, int innerRadius, int row, int column, double angleStart = 0, double angleExtent = CV_2PI, bool angleInDegrees = false);
};

#endif