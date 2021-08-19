#include "ArcElement.hpp"
#include "mathutils.hpp"

using namespace cv;


bool insideArc(int innerRadius, int radius, double angleStart, double angleEnd, double pointNorm, double pointAngle) {
    // Check if radius is in range
    if (pointNorm > radius || pointNorm < innerRadius)
        return false;

    // Check if the angle is in range
    double angleM2Pi = pointAngle - CV_2PI;
    double angleP2Pi = pointAngle + CV_2PI;

    auto between = [](double a, double b, double test) {
        return (a <= test && test < b) || (b < test && test <= a);
    };

    return between(angleStart, angleEnd, angleM2Pi) || between(angleStart, angleEnd, pointAngle) ||
           between(angleStart, angleEnd, angleP2Pi);
}

double ArcElement::findDistance(double firstCoord, double secondCoord) {
    double angleStep = angleExtent / (binCounts.size() - 1);
    double middleRadius = ((double) innerRadius + radius) / 2.0;

    return abs((secondCoord - firstCoord) * angleStep * middleRadius);
}

EdgeResults ArcElement::findEdgePos(const std::vector<double> &coords, SelectType select) {
    // No edges detected
    if (coords.empty())
        return EdgeResults{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    // Find the angle of the 0th coordinate (with respect to profile line)
    double startAngle = angleStart;
    double angleStep = angleExtent / (binCounts.size() - 1);

    // Find the angle of each edge found
    std::vector<double> angles(coords.size());

    for (int i = 0; i < coords.size(); i++)
        // Add 90 degrees to transfer to (r, c) coordinates
        angles[i] = startAngle + (CV_PI / 2) + angleStep * coords[i];

    int centerRow = row;
    int centerColumn = column;
    double middleRadius = (double(innerRadius) + radius) / 2.0;

    // Calculate edgePositions based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = centerColumn + middleRadius * sin(angles.front());
        posEdges.front().y = centerRow + middleRadius * cos(angles.front());
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = centerColumn + middleRadius * sin(angles.back());
        posEdges.back().y = centerRow + middleRadius * cos(angles.back());
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = centerColumn + middleRadius * sin(angles[i]);
            posEdges[i].y = centerRow + middleRadius * cos(angles[i]);
            distances[i - 1] = abs(angles[i] - angles[i - 1]) * middleRadius;
        }
    }

    return EdgeResults{posEdges, distances};
}

// Constructor
ArcElement::ArcElement(int radius_, int innerRadius_, int row_, int column_, double angleStart_, double angleExtent_, bool angleInDegrees)
        : EdgeElement(row_, column_) {
    // Check arguments
    if (radius_ < 0 || innerRadius_ < 0)
        throw std::invalid_argument("Radius values cannot be negative");

    if (innerRadius_ >= radius_)
        throw std::invalid_argument("Inner radius must be less than radius");

    if (angleExtent_ == 0)
        throw std::invalid_argument("angleExtent cannot be 0");

    // Check angle for validness
    if (angleInDegrees) {
        if (angleStart_ <= -180 || angleStart_ > 180)
            throw std::invalid_argument("angleStart must be -180 < angleStart <= 180");

        if (abs(angleExtent_) > 360)
            throw std::invalid_argument("angleExtent must be between -360 and 360");
    } else {
        if (angleStart_ <= -CV_PI || angleStart_ > CV_PI)
            throw std::invalid_argument("angleStart must be -pi < angleStart <= pi");

        if (abs(angleExtent_) > CV_2PI)
            throw std::invalid_argument("angleExtent must be between -2pi and 2pi");
    }

    radius = radius_;
    innerRadius = innerRadius_;

    angleStart = angleStart_;
    angleExtent = angleExtent_;

    if (angleInDegrees) {
        angleStart = deg2rad(angleStart_);
        angleExtent = deg2rad(angleExtent_);
    }

    profileLength = abs(angleExtent) * (double(radius) + innerRadius) / 2.0;

    // Find the bounding box of the annular arc
    // But first in polar coordinates
    double angleEnd = angleStart + angleExtent;

    // First the edges of the annular arc
    std::vector<int> radiusCoords = {innerRadius, radius, innerRadius, radius};
    std::vector<double> thetaCoords = {angleStart, angleStart, angleEnd, angleEnd};

    std::vector<double> rCoords{};
    std::vector<double> cCoords{};

    // Then the endpoints at the 0, 90, 180, 270 angles
    for (int i = -6; i <= 6; i++) {
        double angle = (CV_PI / 2) * i;

        if ((angleStart <= angle && angle < angleEnd) ||
            (angleEnd < angle && angle <= angleStart)) {
            radiusCoords.push_back(radius);
            thetaCoords.push_back(angle);
        }
    }

    // Convert to cartesian coordinates
    for (int i = 0; i < radiusCoords.size(); i++) {
        // Add 90 because we're working with row and column coordinates
        // And angle_start is from the horizontal axis
        rCoords.push_back(radiusCoords[i] * cos(thetaCoords[i] + (CV_PI / 2)));
        cCoords.push_back(radiusCoords[i] * sin(thetaCoords[i] + (CV_PI / 2)));
    }

    minR = (int) *std::min_element(rCoords.cbegin(), rCoords.cend()) - 1;
    maxR = (int) *std::max_element(rCoords.cbegin(), rCoords.cend()) + 1;

    minC = (int) *std::min_element(cCoords.cbegin(), cCoords.cend()) - 1;
    maxC = (int) *std::max_element(cCoords.cbegin(), cCoords.cend()) + 1;

    // Make a 2D array of bin numbers for each (r, c) pair
    int numRows = maxR - minR + 1;
    int numCols = maxC - minC + 1;

    binNumbers = std::vector<int>(size_t(numRows) * numCols);
    int numberOfBins = int(round(abs(angleExtent) * (double(radius) + innerRadius) / 2.0)) + 1;

    binCounts = std::vector<int>(numberOfBins);

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Point2d toProject = Point2d(double(r) + minR, double(c) + minC);

            double pointNorm = norm(toProject);
            double pointAngle = atan2(toProject.y, toProject.x) - (CV_PI / 2);

            // Map to (-pi, pi]
            if (pointAngle <= -CV_PI)
                pointAngle += CV_2PI;

            if (!insideArc(innerRadius, radius, angleStart, angleEnd, pointNorm, pointAngle))
                binNumbers[size_t(r) * numCols + c] = -1;

            else {
                /* Make sure pointAngle is in between (angleStart, angleEnd) so that we can
                   correctly calculate the bin number */
                if (angleExtent > 0 && (pointAngle - angleStart) < 0)
                    pointAngle += CV_2PI;

                else if (angleExtent < 0 && (pointAngle - angleStart) > 0)
                    pointAngle -= CV_2PI;

                // Calculate bin number
                double binNumber = abs((pointAngle - angleStart) / angleExtent) * (double(numberOfBins) - 1);

                // Add the pixel of the image to the correct bin
                int intBinNumber = (int) round(binNumber);

                binNumbers[size_t(r) * numCols + c] = intBinNumber;
                binCounts[intBinNumber]++;
            }
        }
    }
}
