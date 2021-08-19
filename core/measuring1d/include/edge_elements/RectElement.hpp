
#ifndef MEASURING_RECTELEMENT_HPP
#define MEASURING_RECTELEMENT_HPP

#include "EdgeElement.hpp"

/**
 * @brief A measure object for extracting straight edges perpendicular to a rectangle.
 */
class RectangleElement : public virtual EdgeElement {
protected:
    int halfWidth;  // Half of the width of the rectangle
    int halfHeight;  // Half of the height of the rectangle

    double phi;  // Angle of the orientation of the rectangle

    EdgeResults findEdgePos(const std::vector<double> &coords, SelectType select = SelectType::ALL) override;

    double findDistance(double firstCoord, double secondCoord) override;

public:
    /**
     * @brief Construct a RectangleElement object.
     *
     * @param halfWidth Half of the width of the rectangle.
     * @param halfHeight Half of the height of the rectangle.
     * @param row Row coordinate of the center.
     * @param column Column coordinate of the center.
     * @param phi Orientation angle of the rectangle (counterclockwise).
     * @param angleInDegrees Whether phi is in degrees or radians.
     */
    RectangleElement(int halfWidth, int halfHeight, int row, int column, double phi = 0, bool angleInDegrees = false);

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

    /**
     * @brief Get the 4 points of a measureRectangle as a vector of Point2d structs before adjusting to center of mRect.
     * @param measureRectangle
     * @return 4 points of the rectangle.
     */
    [[nodiscard]] std::vector <cv::Point2d> getCornerPoints() const {
        std::vector <cv::Point2d> rectPoints(4);

        int hh = halfHeight;
        int hw = halfWidth;

        // Coordinates before rotating
        /*
        *   [2] ------ 2w ----- [1]
        *   |                    |
        *   2h      (0, 0)       |
        *   |                    |
        *   [3] --------------- [0]
        */

        int rCoords[] = {hh, -hh};
        int cCoords[] = {hw, hw};

        double cosPhi = cos(phi);
        double sinPhi = sin(phi);

        // Rotate points by phi
        for (int i = 0; i < 2; i++) {
            double rotatedR = rCoords[i] * cosPhi - cCoords[i] * sinPhi;
            double rotatedC = rCoords[i] * sinPhi + cCoords[i] * cosPhi;
            rectPoints[i] = cv::Point2d(rotatedR, rotatedC);
        }

        // For the 3rd and 4th points, simply negate the first and second
        for (int i = 2; i <= 3; i++) {
            rectPoints[i] = -rectPoints[size_t(i) - 2];
        }

        return rectPoints;
    }
};


#endif
