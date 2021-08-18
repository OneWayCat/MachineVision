#include "mathutils.hpp"
#include "measureHandle.hpp"

using namespace cv;
using namespace boost::math;

/**
 * Determine if test is between a and b, (a inclusive)
 *
 * @param a
 * @param b
 * @param test
 * @return Whether b is in between a and b.
 */
inline bool between(double a, double b, double test) {
    return (a <= test and test < b) or (b < test and test <= a);
}

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
bool insideArc(int innerRadius, int radius, double angleStart, double angleEnd, double pointNorm, double pointAngle) {
    // Check if radius is in range
    if (pointNorm > radius or pointNorm < innerRadius)
        return false;

    // Check if the angle is in range
    double angleM2Pi = pointAngle - double_constants::two_pi;
    double angleP2Pi = pointAngle + double_constants::two_pi;

    return between(angleStart, angleEnd, angleM2Pi) or between(angleStart, angleEnd, pointAngle) or
           between(angleStart, angleEnd, angleP2Pi);
}

// Constructor
MeasureArc::MeasureArc(int radius_, int innerRadius_, int row_, int column_, double angleStart_,
                       double angleExtent_,
                       bool angleInDegrees) : MeasureHandle(row_, column_) {
    // Check arguments
    if (radius_ < 0 or innerRadius_ < 0)
        throw std::invalid_argument("Radius values cannot be negative");

    if (innerRadius_ >= radius_)
        throw std::invalid_argument("Inner radius out of range");

    if (angleExtent_ == 0)
        throw std::invalid_argument("angleExtent cannot be 0");

    // Check angle for validness
    if (angleInDegrees) {
        if (angleStart_ <= -180 or angleStart_ > 180)
            throw std::invalid_argument("angleStart must be -180 < angleStart <= 180");

        if (abs(angleExtent_) > 360)
            throw std::invalid_argument("angleExtent must be between -360 and 360");
    } else {
        if (angleStart_ <= -double_constants::pi or angleStart_ > double_constants::pi)
            throw std::invalid_argument("angleStart must be -pi < angleStart <= pi");

        if (abs(angleExtent_) > double_constants::two_pi)
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
        double angle = double_constants::half_pi * i;

        if ((angleStart <= angle and angle < angleEnd) or
            (angleEnd < angle and angle <= angleStart)) {
            radiusCoords.push_back(radius);
            thetaCoords.push_back(angle);
        }
    }

    // Convert to cartesian coordinates
    for (int i = 0; i < radiusCoords.size(); i++) {
        // Add 90 because we're working with row and column coordinates
        // And angle_start is from the horizontal axis
        rCoords.push_back(radiusCoords[i] * cos(thetaCoords[i] + double_constants::half_pi));
        cCoords.push_back(radiusCoords[i] * sin(thetaCoords[i] + double_constants::half_pi));
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
            double pointAngle = atan2(toProject.y, toProject.x) - double_constants::half_pi;

            // Map to (-pi, pi]
            if (pointAngle <= -double_constants::pi)
                pointAngle += 2 * double_constants::pi;

            if (not insideArc(innerRadius, radius, angleStart, angleEnd, pointNorm, pointAngle))
                binNumbers[size_t(r) * numCols + c] = -1;

            else {
                /* Make sure pointAngle is in between (angleStart, angleEnd) so that we can
                   correctly calculate the bin number */
                if (angleExtent > 0 and (pointAngle - angleStart) < 0)
                    pointAngle += 2 * double_constants::pi;

                else if (angleExtent < 0 and (pointAngle - angleStart) > 0)
                    pointAngle -= 2 * double_constants::pi;

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

// Constructor
MeasureArcTransposed::MeasureArcTransposed(int radius_, int innerRadius_, int row_, int column_, double angleStart_,
                                           double angleExtent_,
                                           bool angleInDegrees) : MeasureHandle(row_, column_) {
    // Check arguments
    if (radius_ < 0 or innerRadius_ < 0)
        throw std::invalid_argument("Radius values cannot be negative");

    if (innerRadius_ >= radius_)
        throw std::invalid_argument("Inner radius out of range");

    if (angleExtent_ == 0)
        throw std::invalid_argument("angleExtent cannot be 0");

    // Check angle for validness
    if (angleInDegrees) {
        if (angleStart_ <= -180 or angleStart_ > 180)
            throw std::invalid_argument("angleStart must be -180 < angleStart <= 180");

        if (abs(angleExtent_) > 360)
            throw std::invalid_argument("angleExtent must be between -360 and 360");
    } else {
        if (angleStart_ <= -double_constants::pi or angleStart_ > double_constants::pi)
            throw std::invalid_argument("angleStart must be -pi < angleStart <= pi");

        if (abs(angleExtent_) > double_constants::two_pi)
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

    profileLength = abs(radius_ - innerRadius_);

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
        double angle = double_constants::half_pi * i;

        if ((angleStart <= angle and angle < angleEnd) or
            (angleEnd < angle and angle <= angleStart)) {
            radiusCoords.push_back(radius);
            thetaCoords.push_back(angle);
        }
    }

    // Convert to cartesian coordinates
    for (int i = 0; i < radiusCoords.size(); i++) {
        // Add 90 because we're working with row and column coordinates
        // And angle_start is from the horizontal axis
        rCoords.push_back(radiusCoords[i] * cos(thetaCoords[i] + double_constants::half_pi));
        cCoords.push_back(radiusCoords[i] * sin(thetaCoords[i] + double_constants::half_pi));
    }

    minR = (int) *std::min_element(rCoords.cbegin(), rCoords.cend()) - 1;
    maxR = (int) *std::max_element(rCoords.cbegin(), rCoords.cend()) + 1;

    minC = (int) *std::min_element(cCoords.cbegin(), cCoords.cend()) - 1;
    maxC = (int) *std::max_element(cCoords.cbegin(), cCoords.cend()) + 1;

    // Make a 2D array of bin numbers for each (r, c) pair
    int numRows = maxR - minR + 1;
    int numCols = maxC - minC + 1;

    binNumbers = std::vector<int>(size_t(numRows) * numCols);
    int numberOfBins = int(profileLength);

    binCounts = std::vector<int>(numberOfBins);

//    Mat drawImg = Mat::zeros(500, 500, CV_8UC1);
//    circle(drawImg, Point(column_, row_), 1, Scalar(255), -1);
//    imshow("temp", drawImg);

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Point2d toProject = Point2d(double(r) + minR, double(c) + minC);

            double pointNorm = norm(toProject);
            double pointAngle = atan2(toProject.y, toProject.x) - double_constants::half_pi;

            // Map to (-pi, pi]
            if (pointAngle <= -double_constants::pi)
                pointAngle += 2 * double_constants::pi;

            if (not insideArc(innerRadius, radius, angleStart, angleEnd, pointNorm, pointAngle))
                binNumbers[size_t(r) * numCols + c] = -1;

            else {
                // Calculate bin number
                Point trueImgPoint(c + column_ + minC, r + row_ + minR);
                double intBinNumber = (int) round(sqrt(pow(trueImgPoint.x - column_, 2) + pow(trueImgPoint.y - row_, 2)) - innerRadius_);
                if (intBinNumber >= profileLength)
                    intBinNumber = profileLength - 1;
                else if (intBinNumber < 0)
                    intBinNumber = 0;

//                drawImg.at<uchar>(int(trueImgPoint.y), int(trueImgPoint.x)) = 255;

                // Add the pixel of the image to the correct bin
//                int intBinNumber = (int) round(binNumber);

                binNumbers[size_t(r) * numCols + c] = intBinNumber;
                binCounts[intBinNumber]++;
            }
        }
//        imshow("temp", drawImg);
//        waitKey(0);
    }
}