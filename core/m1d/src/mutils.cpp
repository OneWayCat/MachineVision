#include "measuring.hpp"

using namespace cv;
using namespace boost::math;

// Helper functions for creating and drawing rectangles
std::vector<Point2d> getRectPoints(const MeasureRectangle &mRect) {
    std::vector<Point2d> rectPoints(4);

    int hh = mRect.getHalfHeight();
    int hw = mRect.getHalfWidth();

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

    double phi = mRect.getPhi();
    double cosPhi = cos(phi);
    double sinPhi = sin(phi);

    // Rotate points by phi
    for (int i = 0; i < 2; i++) {
        double rotatedR = rCoords[i] * cosPhi - cCoords[i] * sinPhi;
        double rotatedC = rCoords[i] * sinPhi + cCoords[i] * cosPhi;
        rectPoints[i] = Point2d(rotatedR, rotatedC);
    }

    // For the 3rd and 4th points, simply negate the first and second
    for (int i = 2; i <= 3; i++) {
        rectPoints[i] = -rectPoints[size_t(i) - 2];
    }

    return rectPoints;
}

// Helper functions for measuring and fuzzy measuring
Mat nonMaxSuppress(Mat &profile) {
    if (profile.cols < 2)
        return profile;

    Mat absProfile = abs(profile);
    Mat_<double> suppressed = Mat_<double>::zeros(1, profile.cols);

    auto profptr = profile.ptr<double>();
    auto absProfptr = absProfile.ptr<double>();
    auto suppressedPtr = suppressed.ptr<double>();

    // Edge cases (first and last one)
    if (absProfptr[0] > absProfptr[1])
        suppressedPtr[0] = profptr[0];

    int lastIndex = profile.cols - 1;

    if (absProfptr[lastIndex] >= absProfptr[lastIndex - 1])
        suppressedPtr[lastIndex] = profptr[lastIndex];

    // General case
    for (int i = 1; i < profile.cols - 1; i++) {
        if (absProfptr[i] >= absProfptr[i - 1] and absProfptr[i] > absProfptr[i + 1])
            suppressedPtr[i] = profptr[i];
    }

    return suppressed;
}

double findDistance(const MeasureRectangle &measureHandle, double firstCoord, double secondCoord) {
    return secondCoord - firstCoord;
}

double findDistance(const MeasureArc &measureHandle, double firstCoord, double secondCoord) {
    double angleStep = measureHandle.getAngleExtent() / (measureHandle.numberOfBins() - 1);
    double middleRadius = ((double) measureHandle.getInnerRadius() + measureHandle.getRadius()) / 2.0;

    return abs((secondCoord - firstCoord) * angleStep * middleRadius);
}

double findDistance(const MeasureArcTransposed &measureHandle, double firstCoord, double secondCoord) {
    double angleStep = measureHandle.getAngleExtent() / (measureHandle.numberOfBins() - 1);
    double middleRadius = ((double) measureHandle.getInnerRadius() + measureHandle.getRadius()) / 2.0;

    return abs((secondCoord - firstCoord) * angleStep * middleRadius);
}


PosDistanceResult
findEdgePos(const MeasureRectangle &measureHandle, const std::vector<double> &coords, SelectType select) {
    // No edges detected
    if (coords.empty())
        return PosDistanceResult{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    // Add pi/2 to angle because phi is measured from horizontal axis and with the (r, c) coordinate system
    // the horizontal axis is 90 degrees further clockwise
    double angle = measureHandle.getPhi() + double_constants::half_pi;
    double sinTheta = sin(angle);
    double cosTheta = cos(angle);

    // Calculate where the 0th coordinate (with respect to profile line) is
    double rowPos = -measureHandle.getHalfWidth() * cosTheta;
    double colPos = -measureHandle.getHalfWidth() * sinTheta;

    // Adjust for center of measure handle
    rowPos += measureHandle.getRow();
    colPos += measureHandle.getColumn();

    // Calculate position based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = rowPos + coords.front() * cosTheta;
        posEdges.front().y = colPos + coords.front() * sinTheta;
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = rowPos + coords.back() * cosTheta;
        posEdges.back().y = colPos + coords.back() * sinTheta;
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = rowPos + coords[i] * cosTheta;
            posEdges[i].y = colPos + coords[i] * sinTheta;
            distances[i - 1] = coords[i] - coords[i - 1];
        }
    }

    return PosDistanceResult{posEdges, distances};
}

PosDistanceResult
findEdgePos(const MeasureArc &measureHandle, const std::vector<double> &coords, SelectType select) {
    // No edges detected
    if (coords.empty())
        return PosDistanceResult{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    // Find the angle of the 0th coordinate (with respect to profile line)
    double startAngle = measureHandle.getAngleStart();
    double angleStep = measureHandle.getAngleExtent() / (measureHandle.numberOfBins() - 1);

    // Find the angle of each edge found
    std::vector<double> angles(coords.size());

    for (int i = 0; i < coords.size(); i++)
        // Add 90 degrees to transfer to (r, c) coordinates
        angles[i] = startAngle + double_constants::half_pi + angleStep * coords[i];

    int centerRow = measureHandle.getRow();
    int centerColumn = measureHandle.getColumn();
    double middleRadius = (double(measureHandle.getInnerRadius()) + measureHandle.getRadius()) / 2.0;

    // Calculate position based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = centerRow + middleRadius * cos(angles.front());
        posEdges.front().y = centerColumn + middleRadius * sin(angles.front());
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = centerRow + middleRadius * cos(angles.back());
        posEdges.back().y = centerColumn + middleRadius * sin(angles.back());
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = centerRow + middleRadius * cos(angles[i]);
            posEdges[i].y = centerColumn + middleRadius * sin(angles[i]);
            distances[i - 1] = abs(angles[i] - angles[i - 1]) * middleRadius;
        }
    }

    return PosDistanceResult{posEdges, distances};
}

PosDistanceResult
findEdgePos(const MeasureArcTransposed &measureHandle, const std::vector<double> &coords, SelectType select) {
    // No edges detected
    if (coords.empty())
        return PosDistanceResult{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    int centerRow = measureHandle.getRow();
    int centerColumn = measureHandle.getColumn();
    double middleAngle = measureHandle.getAngleStart() + measureHandle.getAngleExtent() / 2 + double_constants::half_pi;

    double cosTheta = cos(middleAngle);
    double sinTheta = sin(middleAngle);

    // Calculate position based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = centerRow + (coords[0] + measureHandle.getInnerRadius()) * cosTheta;
        posEdges.front().y = centerColumn + (coords[0] + measureHandle.getInnerRadius()) * sinTheta;
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = centerRow + (coords[0] + measureHandle.getInnerRadius()) * cosTheta;
        posEdges.back().y = centerColumn + (coords[0] + measureHandle.getInnerRadius()) * sinTheta;
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = centerRow + (coords[i] + measureHandle.getInnerRadius()) * cosTheta;
            posEdges[i].y = centerColumn + (coords[i] + measureHandle.getInnerRadius()) * sinTheta;
            distances[i - 1] = coords[i] - coords[i - 1];
        }
    }

    return PosDistanceResult{posEdges, distances};
}

void validateArgs(const Mat &img, double sigma, double threshold) {
    // Input validation
    // Ensure image is single channel
    CV_Assert(img.depth() == CV_8U and img.channels() == 1);

    // Validate sigma
    if (sigma < 0.4 or sigma > 100)
        throw std::invalid_argument("Sigma must be between 0.4 and 100");

    // Validate threshold
    if (threshold < 0 or threshold > 255)
        throw std::invalid_argument("Threshold must be between 0 and 255");
}

int nextSignedElement(const std::vector<double> &arr, bool sign, int index, bool findStrongest) {
    if (index >= arr.size())
        return -1;

    int first = -1;

    // Find the first element with the given sign
    for (int i = index; i <= arr.size(); i++) {
        if (signbit(arr[i]) == sign) {
            first = i;
            break;
        }
    }

    if (first == -1 or !findStrongest)
        return first;

    // Find the strongest element with given sign (until we reach an element with the opposite sign)
    int i = first;
    double max = 0;
    int maxIndex = 0;

    while (i < arr.size() and signbit(arr[i]) == sign) {
        if (abs(arr[i]) > abs(max)) {
            max = arr[i];
            maxIndex = i;
        }

        i++;
    }

    return maxIndex;
}